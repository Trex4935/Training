// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  public enum PivotState {
    HOME(0.0),
    DOWN(-10.0),
    AGITATE(-95.0),
    STOW(-245.0);

    public final double degrees;
    PivotState(double degrees) {
      this.degrees = degrees;
    }
  }

  private final PivotIO io;
  private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();

  private final Debouncer hardstopDebouncer =
      new Debouncer(PivotConstants.kHardstopDebounceSeconds, DebounceType.kRising);

  private boolean homed = false;
  private boolean seekingHardstop = false;
  private boolean atHardstop = false;
  private double targetDegrees = 0.0;

  public Pivot(PivotIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    boolean pushingHard = inputs.statorCurrentAmps > PivotConstants.kHomingCurrentThresholdAmps;
    boolean stopped = Math.abs(inputs.velocityDegreesPerSecond)
        < PivotConstants.kHomingVelocityThresholdDegreesPerSecond;
    atHardstop = hardstopDebouncer.calculate(seekingHardstop && pushingHard && stopped);

    SmartDashboard.putNumber("Pivot/Position Degrees", inputs.positionDegrees);
    SmartDashboard.putNumber("Pivot/Velocity DegPerSec", inputs.velocityDegreesPerSecond);
    SmartDashboard.putNumber("Pivot/Current", inputs.statorCurrentAmps);
    SmartDashboard.putBoolean("Pivot/Homed", homed);
    SmartDashboard.putNumber("Pivot/Target Degrees", targetDegrees);
  }

  public double getPositionDegrees() {
    return inputs.positionDegrees;
  }

  public boolean isHomed() {
    return homed;
  }

  public void goToPositionDegrees(double degrees) {
    if (!homed) {
      return;
    }
    targetDegrees = MathUtil.clamp(
        degrees,
        PivotConstants.kMinPositionDegrees,
        PivotConstants.kMaxPositionDegrees);
    io.setPositionDegrees(targetDegrees);
  }

  public void zeroEncoder() {
    io.setEncoderPositionDegrees(0.0);
  }

  public void stop() {
    io.stop();
  }
  public void resetZeroAtHardstop() {
    stop();
    io.setEncoderPositionDegrees(0.0);
    homed = true;
    targetDegrees = 0.0;
  }

  public Command home() {
    return Commands.sequence(
        Commands.sequence(
            Commands.run(() -> io.setHomingOutput(PivotConstants.kHomingVolts), this)
                .withTimeout(PivotConstants.kHomingStartupGraceSeconds),
            Commands.runOnce(() -> seekingHardstop = true),
            Commands.run(() -> io.setHomingOutput(PivotConstants.kHomingVolts), this)
                .until(() -> atHardstop))
            .withTimeout(PivotConstants.kHomingTimeoutSeconds),
        Commands.runOnce(() -> {
          io.stop();
          if (atHardstop) {
            zeroEncoder();
            homed = true;
            goToPositionDegrees(PivotState.DOWN.degrees);
          }
        }, this))
        .finallyDo(interrupted -> {
          seekingHardstop = false;
          if (interrupted) {
            io.stop();
          }
        });
  }
}
