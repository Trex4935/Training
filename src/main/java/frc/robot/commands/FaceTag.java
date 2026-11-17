package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import java.util.function.DoubleSupplier;

/** Rotates to center a selected tag while the driver controls translation. */
public class FaceTag extends Command {
  private final Limelight vision;
  private final int tagId;
  private final CommandSwerveDrivetrain drive;
  private final DoubleSupplier forward, left;
  private final PIDController pid = new PIDController(.035, 0, .001);
  private final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

  public FaceTag(Limelight vision, int tagId, CommandSwerveDrivetrain drive, DoubleSupplier forward,
      DoubleSupplier left) {
    this.vision = vision;
    this.tagId = tagId;
    this.drive = drive;
    this.forward = forward;
    this.left = left;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    double tx = vision.getTagYawDegrees(tagId), omega = 0;
    // tx is the camera's horizontal angle to the tag. PID drives this angle toward
    // zero.
    if (Double.isFinite(tx))
      omega = MathUtil.clamp(pid.calculate(tx, 0), -1.0, 1.0);
    drive.setControl(
        request.withVelocityX(forward.getAsDouble()).withVelocityY(left.getAsDouble()).withRotationalRate(omega));
  }

  @Override
  public void end(boolean interrupted) {
    drive.setControl(new SwerveRequest.SwerveDriveBrake());
  }
}
