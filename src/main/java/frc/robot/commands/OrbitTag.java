package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Uses field pose and tag geometry to follow a circle; camera angles alone
 * cannot define radius.
 */
public class OrbitTag extends Command {
  private final CommandSwerveDrivetrain drive;
  private final Supplier<Pose2d> tagPose;
  private final BooleanSupplier valid;
  private final double radius, direction;
  private final PIDController radial = new PIDController(1.0, 0, .02), turn = new PIDController(2, 0, .03);
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

  public OrbitTag(CommandSwerveDrivetrain drive, Supplier<Pose2d> tagPose, BooleanSupplier valid, double radius,
      boolean clockwise) {
    this.drive = drive;
    this.tagPose = tagPose;
    this.valid = valid;
    this.radius = radius;
    this.direction = clockwise ? -1 : 1;
    turn.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    radial.reset();
    turn.reset();
  }

  @Override
  public void execute() {
    Pose2d tag = tagPose.get(), robot = drive.getState().Pose;
    if (!valid.getAsBoolean() || tag == null) {
      stop();
      return;
    }
    Translation2d delta = robot.getTranslation().minus(tag.getTranslation());
    double distance = delta.getNorm();
    if (distance < .05) {
      stop();
      return;
    }
    Translation2d unit = delta.div(distance);
    double radialSpeed = MathUtil.clamp(radial.calculate(distance, radius), -.3, .3);
    Translation2d tangent = tangentialVelocity(unit, direction, .35);
    Translation2d velocity = unit.times(radialSpeed).plus(tangent);
    double omega = MathUtil.clamp(turn.calculate(robot.getRotation().getRadians(), tag.getRotation().getRadians()), -.35, .35);
    drive.setControl(request.withVelocityX(velocity.getX()).withVelocityY(velocity.getY()).withRotationalRate(omega));
  }

  private void stop() {
    drive.setControl(new SwerveRequest.SwerveDriveBrake());
  }

  static Translation2d tangentialVelocity(Translation2d radialUnit, double direction, double speed) {
    return new Translation2d(-radialUnit.getY(), radialUnit.getX()).times(direction * speed);
  }

  @Override
  public void end(boolean interrupted) {
    stop();
  }
}
