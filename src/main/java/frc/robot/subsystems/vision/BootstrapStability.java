package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Pure stability gate for consecutive, fresh MegaTag1 frames. */
public final class BootstrapStability {
  private Pose2d anchor;
  private int frames;
  private double lastTimestamp = Double.NEGATIVE_INFINITY;

  public int observe(Pose2d pose, double timestamp, double now, int tagCount, double ambiguity,
      double distanceMeters, double speedMps, double omegaRadPerSec) {
    boolean stationary = Math.abs(speedMps) <= 0.15 && Math.abs(omegaRadPerSec) <= 0.15;
    boolean fresh = Double.isFinite(timestamp) && timestamp > lastTimestamp && now - timestamp <= 0.25
        && now - timestamp >= -0.05;
    boolean multiSafe = tagCount >= 2;
    boolean singleSafe = tagCount == 1 && Double.isFinite(ambiguity) && ambiguity >= 0
        && ambiguity <= 0.08 && Double.isFinite(distanceMeters) && distanceMeters > 0
        && distanceMeters <= 3.0;
    if (!stationary || !fresh || pose == null || (!multiSafe && !singleSafe)) { reset(); return frames; }
    lastTimestamp = timestamp;
    if (anchor == null || anchor.getTranslation().getDistance(pose.getTranslation()) > 0.15
        || Math.abs(anchor.getRotation().minus(pose.getRotation()).getDegrees()) > 4.0) {
      anchor = pose;
      frames = 1;
    } else frames++;
    return frames;
  }

  public int requiredFrames(int tagCount) { return tagCount >= 2 ? 5 : 12; }
  public void reset() { anchor = null; frames = 0; }
}
