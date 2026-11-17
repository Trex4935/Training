package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Converts a tag-local destination into a blue-origin robot-center pose.
 * Offsets are tag-local meters.
 */
public final class TagApproach {
  private TagApproach() {
  }

  public enum Mount {
    WALL, OVERHEAD
  }

  public record Config(int tagId, double forwardMeters, double leftMeters, double headingRelativeRadians,
      double robotLengthMeters, double robotWidthMeters, double clearanceMeters, Mount mount) {
    public Config {
      if (tagId <= 0 || mount == null
          || !finite(forwardMeters, leftMeters, headingRelativeRadians, robotLengthMeters, robotWidthMeters,
              clearanceMeters)
          || robotLengthMeters <= 0 || robotWidthMeters <= 0 || clearanceMeters < 0)
        throw new IllegalArgumentException("Invalid tag target configuration");
    }
  }

  /**
   * Null means the selected tag has no authoritative map pose; callers must stop.
   */
  public static Pose2d target(Pose3d tagFieldPose, Config config) {
    if (tagFieldPose == null || config == null)
      return null;
    double safeRadius = Math.hypot(config.robotLengthMeters() / 2, config.robotWidthMeters() / 2)
        + config.clearanceMeters();
    double horizontalOffset = Math.hypot(config.forwardMeters(), config.leftMeters());
    if (config.mount() == Mount.WALL && config.forwardMeters() > -safeRadius)
      return null;
    if (config.mount() == Mount.OVERHEAD && horizontalOffset < safeRadius)
      return null;
    // A Transform3d expresses the desired point in the tag's own axes, then rotates
    // it into field axes.
    Transform3d relative = new Transform3d(new Translation3d(config.forwardMeters(), config.leftMeters(), 0),
        new Rotation3d());
    Pose3d point = tagFieldPose.transformBy(relative);
    Rotation2d heading = tagFieldPose.getRotation().toRotation2d()
        .plus(Rotation2d.fromRadians(config.headingRelativeRadians()));
    // The robot drives on the floor; an overhead tag's height is deliberately
    // projected away.
    return new Pose2d(point.getX(), point.getY(), heading);
  }

  private static boolean finite(double... values) {
    for (double v : values)
      if (!Double.isFinite(v))
        return false;
    return true;
  }
}
