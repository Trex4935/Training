// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {


  @Override
  public void periodic() {
    io.updateInputs(inputs);

    boolean pushingHard = inputs.statorCurrentAmps > PivotConstants.kHomingCurrentThresholdAmps;
    boolean stopped = Math.abs(inputs.velocityDegreesPerSecond)
        < PivotConstants.kHomingVelocityThresholdDegreesPerSecond;
    atHardstop = hardstopDebouncer.calculate(seekingHardstop && pushingHard && stopped);
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