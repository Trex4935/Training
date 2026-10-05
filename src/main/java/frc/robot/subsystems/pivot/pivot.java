// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Named positions, in degrees from the hardstop (same values as the 2026 intake pivot). */
  public enum PivotState {
    HOME(0.0),
    DOWN(-10.0),
    AGITATE(-95.0),
    STOW(-245.0);

    public final double rotations;

    PivotState(double degrees) {
      this.rotations = Units.degreesToRotations(degrees);
    }
  }

  private final PivotIO io;
  private final PivotIO.PivotIOInputs inputs = new PivotIO.PivotIOInputs();

  /** The hardstop condition has to stay true for a moment, so a quick current spike is ignored. */
  private final Debouncer hardstopDebouncer =
      new Debouncer(PivotConstants.kHardstopDebounceSeconds, DebounceType.kRising);

  /** True once the encoder has been zeroed at the hardstop. Until then positions mean nothing. */
  private boolean homed = false;
  /** True while homing is actively looking for the hardstop. */
  private boolean seekingHardstop = false;
  private boolean atHardstop = false;
  private double targetRotations = 0.0;
  private double lastTestTarget = PivotState.DOWN.rotations;

  public Pivot(PivotIO io) {
    this.io = io;
    SmartDashboard.putNumber("Pivot/Test Target", lastTestTarget);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    boolean pushingHard = inputs.statorCurrentAmps > PivotConstants.kHomingCurrentThresholdAmps;
    boolean stopped =
        Math.abs(inputs.velocityRotationsPerSecond) < PivotConstants.kHomingVelocityThreshold;
    atHardstop = hardstopDebouncer.calculate(seekingHardstop && pushingHard && stopped);

    SmartDashboard.putNumber("Pivot/Position", inputs.positionRotations);
    SmartDashboard.putNumber("Pivot/Velocity", inputs.velocityRotationsPerSecond);
    SmartDashboard.putNumber("Pivot/Current", inputs.statorCurrentAmps);
    SmartDashboard.putBoolean("Pivot/Homed", homed);
    SmartDashboard.putNumber("Pivot/Target", targetRotations);
  }

  public double getPositionRotations() {
    return inputs.positionRotations;
  }

  public boolean isHomed() {
    return homed;
  }

  /** Moves the pivot to a position with Motion Magic. Ignored until the pivot has homed. */
  public void goToPosition(double rotations) {
    if (!homed) {
      // Before homing, the encoder's 0 is wherever the pivot was at power-on, not the hardstop.
      return;
    }
    targetRotations = MathUtil.clamp(
        rotations,
        PivotConstants.kMinPositionRotations,
        PivotConstants.kMaxPositionRotations);
    io.setPosition(targetRotations);
  }

  /** Sets the current position as zero. Only call this when the pivot is on the hardstop. */
  public void zeroEncoder() {
    io.setEncoderPosition(0.0);
  }

  public void stop() {
    io.stop();
  }

  /** Command: Motion Magic to a named position. */
  public Command goTo(PivotState state) {
    return Commands.runOnce(() -> goToPosition(state.rotations), this);
  }

  /**
   * Command: whenever the "Pivot/Test Target" number (rotations) on the dashboard changes, Motion
   * Magic to it. Used as the pivot's default command, so it only runs when no other command (like
   * homing or a button) is using the pivot.
   */
  public Command followTestTarget() {
    return Commands.run(() -> {
      double testTarget = SmartDashboard.getNumber("Pivot/Test Target", lastTestTarget);
      if (testTarget != lastTestTarget) {
        lastTestTarget = testTarget;
        goToPosition(testTarget);
      }
    }, this);
  }

  /**
   * Command: find the hardstop and zero the encoder there.
   *
   * <ol>
   *   <li>Drive slowly toward the hardstop. For the first moment, don't look for the hardstop:
   *       a motor that is just starting draws high current while barely moving, like a stall.
   *   <li>Keep driving until we detect the hardstop (or time out).
   *   <li>Stop. If we really found the hardstop: zero the encoder, mark homed, Motion Magic to DOWN.
   *       If we timed out: stay stopped and do NOT zero, because we don't know where we are.
   * </ol>
   */
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
            goToPosition(PivotState.DOWN.rotations);
          }
        }, this))
        .finallyDo(interrupted -> {
          seekingHardstop = false;
          // Interrupted mid-seek (button press, robot disabled...): don't leave the homing voltage
          // running. Not on normal finish, or we would cancel the Motion Magic move to DOWN.
          if (interrupted) {
            io.stop();
          }
        });
  }
}
