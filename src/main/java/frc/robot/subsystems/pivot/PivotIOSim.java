// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;


public class PivotIOSim implements PivotIO {
  private static final double kHardstopDegrees = 0.0;
  private static final double kStartingDegrees = -45.0; 
  private static final double kHomingDegreesPerSecondPerVolt = 30.0;
  private static final double kHardstopCurrentAmps = 80.0;
  private static final double kMovingCurrentAmps = 5.0;
  private static final double kLoopSeconds = 0.02;

  private double trueDegrees = kStartingDegrees;
  private double encoderOffsetDegrees = kStartingDegrees;
  private double homingVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    double velocity = 0.0;
    double current = 0.0;

    if (homingVolts != 0.0) {
      if (trueDegrees >= kHardstopDegrees) {
        current = kHardstopCurrentAmps;
      } else {
        velocity = homingVolts * kHomingDegreesPerSecondPerVolt;
        trueDegrees = Math.min(trueDegrees + velocity * kLoopSeconds, kHardstopDegrees);
        current = kMovingCurrentAmps;
      }
    }

    inputs.positionDegrees = trueDegrees - encoderOffsetDegrees;
    inputs.velocityDegreesPerSecond = velocity;
    inputs.statorCurrentAmps = current;
  }

  @Override
  public void setPositionDegrees(double degrees) {
    homingVolts = 0.0;
    trueDegrees = Math.min(degrees + encoderOffsetDegrees, kHardstopDegrees);
  }

  @Override
  public void setHomingOutput(double volts) {
    homingVolts = volts;
  }

  @Override
  public void setEncoderPositionDegrees(double degrees) {
    encoderOffsetDegrees = trueDegrees - degrees;
  }

  @Override
  public void stop() {
    homingVolts = 0.0;
  }
}
