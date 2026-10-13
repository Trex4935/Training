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

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private static final double kMaxAngularRateRadiansPerSecond = 2 * Math.PI;
  private static final double kMaxMeasurementAgeSeconds = 0.5;
  private static final double kFutureTimestampToleranceSeconds = 0.05;
  private final String[] cams;
  private final Map<String, Double> lastAcceptedTimestamp = new HashMap<>();
  private long acceptedCount;
  private long rejectedCount;

  public Limelight(String... names) {
    this.cams = names;
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
      LimelightHelpers.SetRobotOrientation(name, robotPose.getRotation().getDegrees(),
          Math.toDegrees(angularRateRadiansPerSecond), 0, 0, 0, 0);
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
      double age = Timer.getFPGATimestamp() - timestamp;
      if (!Double.isFinite(timestamp) || timestamp <= 0
          || age > kMaxMeasurementAgeSeconds
          || age < -kFutureTimestampToleranceSeconds
          || timestamp <= lastAcceptedTimestamp.getOrDefault(name, 0.0)) {
        reject(name, "Stale or invalid timestamp");
        continue;
      }

      double trust = MT2.avgTagDist / MT2.tagCount;
      double dev = .1 * trust;
      result.add(new Measurement(MT2, VecBuilder.fill(dev, dev, 999999)));
      lastAcceptedTimestamp.put(name, timestamp);
      acceptedCount++;
      SmartDashboard.putString("Vision/" + name + "/Status", "Accepted");
      SmartDashboard.putNumber("Vision/" + name + "/LastAcceptedTimestamp", timestamp);
    }
    SmartDashboard.putNumber("Vision/AcceptedCount", acceptedCount);
    SmartDashboard.putNumber("Vision/RejectedCount", rejectedCount);
    return result;
  }

  private void reject(String name, String reason) {
    rejectedCount++;
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
    // This method will be called once per scheduler run
  }
}
