// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.vision.BootstrapStability;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import java.util.Optional;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.VisionConstants;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private static final double kMaxAngularRateRadiansPerSecond = 2 * Math.PI;
  private static final double kMaxMeasurementAgeSeconds = 0.5;
  private static final double kFutureTimestampToleranceSeconds = 0.05;
  private final String[] cams;
  private final Map<String, Double> lastAcceptedTimestamp = new HashMap<>();
  private long acceptedCount;
  private long rejectedCount;
  // Set true only after confirming the named camera is an LL4 with its IMU configured.
  private static final boolean kHasLimelight4Imu = true;
  private boolean fieldHeadingInitialized;
  private int currentImuMode = -1;
  private double lastHeadingSeedTime = Double.NEGATIVE_INFINITY;
  private String lastRejectionReason = "None";
  private double latestFieldHeadingDegrees;
  private double latestAcceptedPoseTimestamp;
  private double lastVisionX, lastVisionY, lastVisionHeading;
  private int stableBootstrapFrames;
  private final BootstrapStability bootstrapStability = new BootstrapStability();
  private Pose2d bootstrapAnchor;
  private String seedSource = "NONE";
  private double lastBootstrapTimestamp;
  private static final int kBootstrapFramesSingle = 12;
  private static final int kBootstrapFramesMulti = 5;
  private static final double kSingleMaxAmbiguity = 0.08;
  private static final double kSingleMaxDistanceMeters = 3.0;
  private static final double kBootstrapMaxPositionChange = 0.15;
  private static final double kBootstrapMaxHeadingChangeDeg = 4.0;
  private static final double kImuReseedSeconds = 0.50;
  private static final double kTargetMaxAgeSeconds = 0.25;
  private final Map<Integer, TargetObservation> observations = new HashMap<>();
  // Limelight does not publish its uploaded .fmap as a field layout. Keep this JSON export
  // synchronized with that exact camera map; absent or invalid data means no field target.
  private final AprilTagFieldLayout fieldLayout = loadSynchronizedFieldLayout();

  public Limelight(String... names) {
    this.cams = names;
  }

  /** Marks a known absolute field heading; driver-forward calibration must not call this. */
  public void initializeFieldHeading(String reason) {
    fieldHeadingInitialized = true;
    lastHeadingSeedTime = Timer.getFPGATimestamp();
    seedSource = reason;
    SmartDashboard.putString("Vision/HeadingChangeReason", reason);
  }
  public String getSeedSource() { return seedSource; }
  public boolean hasTrustedFieldPose() { return isFieldPoseFresh(); }
  public boolean isFieldPoseFresh() { return fieldHeadingInitialized && Timer.getFPGATimestamp()-latestAcceptedPoseTimestamp <= kMaxMeasurementAgeSeconds; }
  public boolean hasTag(int id) { return getTargetObservation(id).valid(); }
  public double getTagYawDegrees(int id) { return getTargetObservation(id).yawDegrees(); }
  public double getTagPitchDegrees(int id) { return getTargetObservation(id).pitchDegrees(); }
  public Pose2d getTagFieldPose(int id) {
    if(fieldLayout==null)return null;
    Optional<edu.wpi.first.math.geometry.Pose3d> pose = fieldLayout.getTagPose(id);
    return pose.map(p -> p.toPose2d()).orElse(null);
  }
  public edu.wpi.first.math.geometry.Pose3d getTagFieldPose3d(int id) { return fieldLayout==null?null:fieldLayout.getTagPose(id).orElse(null); }
  private static AprilTagFieldLayout loadSynchronizedFieldLayout() {
    try {
      AprilTagFieldLayout layout=new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath().resolve(VisionConstants.kTrainingMapFile));
      if(layout.getTagPose(VisionConstants.kTrainingTagId).isEmpty()) {
        SmartDashboard.putString("Vision/FieldMapStatus","REJECTED: Tag 28 missing from Java layout");
        return null;
      }
      SmartDashboard.putString("Vision/FieldMapStatus","Loaded hallway layout with Tag 28");
      return layout;
    } catch(Exception e) { SmartDashboard.putString("Vision/FieldMapStatus","MISSING_OR_INVALID "+VisionConstants.kTrainingMapFile); return null; }
  }
  public TargetObservation getTargetObservation(int id) {
    TargetObservation obs = observations.get(id);
    if (obs == null) return TargetObservation.invalid(id);
    boolean fresh = obs.valid() && Timer.getFPGATimestamp() - obs.timestampSeconds() <= kTargetMaxAgeSeconds;
    return fresh ? obs : TargetObservation.invalid(id);
  }
  public boolean hasFreshFieldPose() { return isFieldPoseFresh(); }

  public record TargetObservation(int id, boolean valid, double yawDegrees, double pitchDegrees, double timestampSeconds) {
    static TargetObservation invalid(int id) { return new TargetObservation(id, false, Double.NaN, Double.NaN, Double.NaN); }
  }

  /** Poll MegaTag1 for a stable absolute pose. The caller applies the seed once and reports it here. */
  public Pose2d getBootstrapPose(double speedMps, double omegaRadPerSec) {
    if (fieldHeadingInitialized || Math.abs(speedMps) > 0.15 || Math.abs(omegaRadPerSec) > 0.15) {
      stableBootstrapFrames = 0; bootstrapAnchor = null; publishBootstrap("WAITING_FOR_TRUSTED_VISION", 0, Double.NaN);
      return null;
    }
    for (String camera : cams) {
      PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(camera);
      if (estimate == null || !LimelightHelpers.validPoseEstimate(estimate) || estimate.pose == null
          || estimate.tagCount <= 0 || !Double.isFinite(estimate.timestampSeconds)
          || Timer.getFPGATimestamp() - estimate.timestampSeconds > 0.25
          || estimate.timestampSeconds <= lastBootstrapTimestamp) continue;
      double ambiguity = 1.0;
      if (estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
        ambiguity = 0.0;
        for (var fiducial : estimate.rawFiducials) ambiguity = Math.max(ambiguity, fiducial.ambiguity);
      }
      boolean singleSafe = estimate.tagCount > 1 || (ambiguity >= 0 && ambiguity <= kSingleMaxAmbiguity
          && estimate.avgTagDist > 0 && estimate.avgTagDist <= kSingleMaxDistanceMeters);
      SmartDashboard.putNumber("Vision/Bootstrap/TagCount", estimate.tagCount);
      SmartDashboard.putNumber("Vision/Bootstrap/Ambiguity", ambiguity);
      if (!singleSafe) { stableBootstrapFrames = 0; bootstrapAnchor = null; continue; }
      stableBootstrapFrames = bootstrapStability.observe(estimate.pose, estimate.timestampSeconds,
          Timer.getFPGATimestamp(), estimate.tagCount, ambiguity, estimate.avgTagDist, speedMps, omegaRadPerSec);
      bootstrapAnchor = estimate.pose;
      lastBootstrapTimestamp = estimate.timestampSeconds;
      int required = bootstrapStability.requiredFrames(estimate.tagCount);
      publishBootstrap("BOOTSTRAPPING", estimate.tagCount, ambiguity);
      if (stableBootstrapFrames >= required) {
        seedSource = estimate.tagCount > 1 ? "MEGATAG1_MULTI_TAG" : "MEGATAG1_SINGLE_TAG_STABLE";
        publishBootstrap("INITIALIZED", estimate.tagCount, ambiguity);
        return estimate.pose;
      }
    }
    if (stableBootstrapFrames == 0) publishBootstrap("WAITING_FOR_TRUSTED_VISION", 0, Double.NaN);
    return null;
  }

  private void publishBootstrap(String state, int tags, double ambiguity) {
    SmartDashboard.putString("Vision/Bootstrap/State", state);
    SmartDashboard.putNumber("Vision/Bootstrap/TagCount", tags);
    SmartDashboard.putNumber("Vision/Bootstrap/Ambiguity", ambiguity);
    SmartDashboard.putNumber("Vision/Bootstrap/StabilityFrames", stableBootstrapFrames);
  }

  public List<Measurement> getMeasurement(Pose2d robotPose, double angularRateRadiansPerSecond) {
    List<Measurement> result = new ArrayList<>();
    for (String name : cams) {
      if (robotPose == null || !Double.isFinite(robotPose.getRotation().getDegrees())
          || !Double.isFinite(angularRateRadiansPerSecond)) {
        reject(name, "Invalid robot orientation");
        continue;
      }

      // MegaTag2 expects the current blue-origin yaw and yaw rate before reading its pose.
      latestFieldHeadingDegrees = robotPose.getRotation().getDegrees();
      LimelightHelpers.SetRobotOrientation(name, robotPose.getRotation().getDegrees(),
          Math.toDegrees(angularRateRadiansPerSecond), 0, 0, 0, 0);
      // IMU mode 1 continually seeds an LL4; it does not establish global field heading.
      int desiredMode = (!fieldHeadingInitialized || edu.wpi.first.wpilibj.DriverStation.isDisabled()
          || Timer.getFPGATimestamp() - lastHeadingSeedTime < 0.50) ? 1 : 4;
      if (kHasLimelight4Imu && name.equals("limelight-bow") && desiredMode != currentImuMode) {
        LimelightHelpers.SetIMUMode(name, desiredMode);
        currentImuMode = desiredMode;
      }
      if (!fieldHeadingInitialized) { reject(name, "Global field heading unknown"); continue; }
      if (Math.abs(angularRateRadiansPerSecond) > kMaxAngularRateRadiansPerSecond) {
        reject(name, "Angular rate too high");
        continue;
      }

      PoseEstimate MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      if (MT2 == null || !MT2.isMegaTag2 || !LimelightHelpers.validPoseEstimate(MT2)
          || MT2.tagCount <= 0) {
        reject(name, "No valid MegaTag2 tags");
        continue;
      }
      if (MT2.pose == null || !Double.isFinite(MT2.pose.getX())
          || !Double.isFinite(MT2.pose.getY())
          || !Double.isFinite(MT2.pose.getRotation().getRadians())
          || MT2.pose.getX() == 0 || MT2.pose.getY() == 0
          || !Double.isFinite(MT2.avgTagDist) || MT2.avgTagDist <= 0
          || MT2.avgTagDist >= 5) {
        reject(name, "Pose or tag distance rejected");
        continue;
      }

      double timestamp = MT2.timestampSeconds;
      double poseJump = robotPose.getTranslation().getDistance(MT2.pose.getTranslation());
      double age = Timer.getFPGATimestamp() - timestamp;
      if (!Double.isFinite(timestamp) || timestamp <= 0
          || age > kMaxMeasurementAgeSeconds
          || age < -kFutureTimestampToleranceSeconds
          || timestamp <= lastAcceptedTimestamp.getOrDefault(name, 0.0)) {
        reject(name, "Stale or invalid timestamp");
        continue;
      }
      // MegaTag2 depends on supplied heading; a large one-frame jump usually means a bad solve.
      if (poseJump > 2.5) { reject(name, "Pose jump too large"); continue; }

      double trust = MT2.avgTagDist / MT2.tagCount;
      double dev = .1 * trust;
      result.add(new Measurement(MT2, VecBuilder.fill(dev, dev, 999999)));
      lastAcceptedTimestamp.put(name, timestamp);
      latestAcceptedPoseTimestamp = Math.max(latestAcceptedPoseTimestamp, timestamp);
      acceptedCount++;
      lastVisionX = MT2.pose.getX(); lastVisionY = MT2.pose.getY();
      lastVisionHeading = MT2.pose.getRotation().getDegrees();
      SmartDashboard.putString("Vision/" + name + "/Status", "Accepted");
      SmartDashboard.putNumber("Vision/" + name + "/LastAcceptedTimestamp", timestamp);
    }
    SmartDashboard.putNumber("Vision/AcceptedCount", acceptedCount);
    SmartDashboard.putNumber("Vision/RejectedCount", rejectedCount);
    SmartDashboard.putNumber("Vision/EstimatedFieldHeadingDeg", latestFieldHeadingDegrees);
    SmartDashboard.putBoolean("Vision/FieldHeadingInitialized", fieldHeadingInitialized);
    SmartDashboard.putNumber("Vision/LimelightIMUMode", currentImuMode);
    SmartDashboard.putString("Vision/LastRejectionReason", lastRejectionReason);
    return result;
  }

  private void reject(String name, String reason) {
    rejectedCount++;
    lastRejectionReason = reason;
    SmartDashboard.putString("Vision/" + name + "/Status", reason);
  }

  public static class Measurement {
    public final PoseEstimate poseEstimate;
    public final Matrix<N3, N1> standardDeviations;

    public Measurement(PoseEstimate MT2, Matrix<N3, N1> standardDeviations) {
      this.poseEstimate = MT2;
      this.standardDeviations = standardDeviations;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Vision/PoseX", lastVisionX);
    SmartDashboard.putNumber("Vision/PoseY", lastVisionY);
    SmartDashboard.putNumber("Vision/PoseHeadingDeg", lastVisionHeading);
    for (String name : cams) {
      SmartDashboard.putBoolean("Vision/" + name + "/Connected", LimelightHelpers.getTV(name));
      SmartDashboard.putNumber("Vision/" + name + "/TagID", LimelightHelpers.getFiducialID(name));
    }
    updateTargetObservations();
  }

  private void updateTargetObservations() {
    double now = Timer.getFPGATimestamp();
    for (String camera : cams) {
      var targeting = LimelightHelpers.getLatestResults(camera);
      if (targeting == null) continue;
      if (!targeting.valid || targeting.targets_Fiducials == null) continue;
      double timestamp = targeting.timestamp_RIOFPGA_capture;
      for (var target : targeting.targets_Fiducials) {
        int id = (int) target.fiducialID;
        if (id <= 0 || !Double.isFinite(target.tx) || !Double.isFinite(target.ty)) continue;
        if (!Double.isFinite(timestamp) || timestamp <= 0 || now - timestamp > kTargetMaxAgeSeconds || timestamp > now + kFutureTimestampToleranceSeconds) continue;
        TargetObservation previous = observations.get(id);
        if (previous == null || timestamp > previous.timestampSeconds())
          observations.put(id, new TargetObservation(id, true, target.tx, target.ty, timestamp));
      }
    }
  }
}
