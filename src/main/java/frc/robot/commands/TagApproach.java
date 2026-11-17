package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Computes a safe robot-center goal from a configured tag pose. Tag +X points away from its face. */
public final class TagApproach {
  private TagApproach() {}
  public static Pose2d target(Pose2d tagPose, double standoffMeters, double robotFrontExtentMeters,
      double clearanceMeters) {
    if (tagPose == null || standoffMeters < 0 || robotFrontExtentMeters < 0 || clearanceMeters < 0)
      throw new IllegalArgumentException("Invalid tag approach geometry");
    double distance = Math.max(standoffMeters, robotFrontExtentMeters + clearanceMeters);
    Translation2d robotCenter = tagPose.getTranslation().plus(
        new Translation2d(-distance, 0).rotateBy(tagPose.getRotation()));
    return new Pose2d(robotCenter, tagPose.getRotation());
  }
}
