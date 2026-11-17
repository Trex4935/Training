package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.function.Consumer;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Field-relative PID example: each controller turns a field position error into
 * a velocity.
 */
public class DriveToTagPose extends Command {
  private final CommandSwerveDrivetrain drive;
  private final Supplier<Pose2d> target;
  private final BooleanSupplier valid;
  private final PIDController x = new PIDController(0.8, 0, 0.02);
  private final PIDController y = new PIDController(0.8, 0, 0.02);
  private final PIDController theta = new PIDController(2.0, 0, 0.04);
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();
  public static final double MAX_SPEED = 0.5, MAX_OMEGA = 0.4;
  private static final double POSITION_TOLERANCE = 0.08, HEADING_TOLERANCE = Math.toRadians(4);
  private boolean finished;

  public DriveToTagPose(CommandSwerveDrivetrain drive, Supplier<Pose2d> target, BooleanSupplier valid) {
    this.drive = drive;
    this.target = target;
    this.valid = valid;
    theta.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    x.reset();
    y.reset();
    theta.reset();
    finished = false;
  }

  @Override
  public void execute() {
    Pose2d goal = target.get(), pose = drive.getState().Pose;
    if (!valid.getAsBoolean() || goal == null) {
      stop();
      return;
    }
    double ex = goal.getX() - pose.getX(), ey = goal.getY() - pose.getY();
    double et = goal.getRotation().minus(pose.getRotation()).getRadians();
    if (Math.hypot(ex, ey) <= POSITION_TOLERANCE && Math.abs(et) <= HEADING_TOLERANCE) {
      stop();
      finished = true;
      return;
    }
    // The estimator and goal use blue-origin field X/Y, so these are field-relative
    // speeds.
    drive.setControl(request.withVelocityX(MathUtil.clamp(x.calculate(pose.getX(), goal.getX()), -MAX_SPEED, MAX_SPEED))
        .withVelocityY(MathUtil.clamp(y.calculate(pose.getY(), goal.getY()), -MAX_SPEED, MAX_SPEED))
        .withRotationalRate(MathUtil.clamp(
            theta.calculate(pose.getRotation().getRadians(), goal.getRotation().getRadians()), -MAX_OMEGA, MAX_OMEGA)));
  }

  private void stop() {
    brake(drive::setControl);
  }

  static void brake(Consumer<SwerveRequest> output) { output.accept(new SwerveRequest.SwerveDriveBrake()); }

  @Override
  public void end(boolean interrupted) {
    stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
