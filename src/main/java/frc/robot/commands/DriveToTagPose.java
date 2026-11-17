package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Low speed holonomic approach; it stops output immediately if localization or tag validity is lost. */
public class DriveToTagPose extends Command {
  private final CommandSwerveDrivetrain drive;
  private final Supplier<Pose2d> target;
  private final BooleanSupplier valid;
  private final PIDController x = new PIDController(0.8, 0, 0.02);
  private final PIDController y = new PIDController(0.8, 0, 0.02);
  private final PIDController theta = new PIDController(2.0, 0, 0.04);
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();
  private static final double MAX_V = 0.6, MAX_OMEGA = 0.5;

  public DriveToTagPose(CommandSwerveDrivetrain drive, Supplier<Pose2d> target, BooleanSupplier valid) {
    this.drive = drive; this.target = target; this.valid = valid;
    theta.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }
  @Override public void initialize() { x.reset(); y.reset(); theta.reset(); }
  @Override public void execute() {
    Pose2d goal = target.get();
    Pose2d pose = drive.getState().Pose;
    if (!valid.getAsBoolean() || goal == null) { stop("STOPPED: target/localization invalid", pose, goal); return; }
    double vx = MathUtil.clamp(x.calculate(pose.getX(), goal.getX()), -MAX_V, MAX_V);
    double vy = MathUtil.clamp(y.calculate(pose.getY(), goal.getY()), -MAX_V, MAX_V);
    double omega = MathUtil.clamp(theta.calculate(pose.getRotation().getRadians(), goal.getRotation().getRadians()), -MAX_OMEGA, MAX_OMEGA);
    drive.setControl(request.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
    SmartDashboard.putString("Vision/DriveToTag/State", "ALIGNING");
    SmartDashboard.putNumber("Vision/DriveToTag/TargetX", goal.getX());
    SmartDashboard.putNumber("Vision/DriveToTag/TargetY", goal.getY());
    SmartDashboard.putNumber("Vision/DriveToTag/PositionError", pose.getTranslation().getDistance(goal.getTranslation()));
    SmartDashboard.putNumber("Vision/DriveToTag/HeadingErrorDeg", goal.getRotation().minus(pose.getRotation()).getDegrees());
  }
  private void stop(String state, Pose2d pose, Pose2d goal) {
    drive.setControl(new SwerveRequest.SwerveDriveBrake());
    SmartDashboard.putString("Vision/DriveToTag/State", state);
    if (goal != null) SmartDashboard.putNumber("Vision/DriveToTag/PositionError", pose.getTranslation().getDistance(goal.getTranslation()));
  }
  @Override public void end(boolean interrupted) { drive.setControl(new SwerveRequest.SwerveDriveBrake()); SmartDashboard.putString("Vision/DriveToTag/State", "STOPPED"); }
  @Override public boolean isFinished() { return false; }
}
