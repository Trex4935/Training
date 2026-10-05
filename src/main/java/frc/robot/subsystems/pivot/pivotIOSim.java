// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A very simple fake pivot for simulation. No physics: Motion Magic is approximated as "move toward
 * the target at a steady speed", and homing is "drift toward the hardstop until we hit it".
 *
 * <p>Like a real TalonFX, the encoder reads 0.0 at startup even though the pivot starts away from
 * the hardstop (at STOW). That is exactly why we have to home.
 *
 * <p>The numbers here only make the simulation look reasonable. They are NOT robot constants.
 */
public class PivotIOSim implements PivotIO {
  private static final double kLoopSeconds = 0.02;
  /** Where the pivot really starts, measured from the hardstop (STOW). */
  private static final double kStartPositionRotations = Units.degreesToRotations(-245.0);
  /** Fake Motion Magic speed. */
  private static final double kMotionMagicSpeedRotationsPerSecond = 1.5;
  /** Fake homing speed per volt (the real pivot does about 500 deg/s at 3 V). */
  private static final double kRotationsPerSecondPerVolt = Units.degreesToRotations(500.0) / 3.0;
  /** Fake current while pushing into the hardstop (must be above the 65 A homing threshold). */
  private static final double kStallCurrentAmps = 80.0;
  private static final double kMovingCurrentAmps = 5.0;

  /** True position of the pivot. 0.0 is the hardstop; the pivot lives at negative positions. */
  private double truePositionRotations = kStartPositionRotations;
  /** encoder reading = true position - encoderOffset. */
  private double encoderOffsetRotations = kStartPositionRotations;

  private boolean usingMotionMagic = false;
  private double targetRotations = 0.0;
  private double homingVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    double velocity = 0.0;

    if (DriverStation.isEnabled()) {
      if (usingMotionMagic) {
        double trueTarget = targetRotations + encoderOffsetRotations;
        double maxStep = kMotionMagicSpeedRotationsPerSecond * kLoopSeconds;
        velocity = MathUtil.clamp(trueTarget - truePositionRotations, -maxStep, maxStep) / kLoopSeconds;
      } else {
        velocity = homingVolts * kRotationsPerSecondPerVolt;
      }
    }

    truePositionRotations += velocity * kLoopSeconds;

    // The hardstop: the pivot cannot go above 0.0.
    boolean pushingIntoHardstop = truePositionRotations >= 0.0 && velocity > 0.0;
    if (pushingIntoHardstop) {
      truePositionRotations = 0.0;
      velocity = 0.0;
    }

    inputs.positionRotations = truePositionRotations - encoderOffsetRotations;
    inputs.velocityRotationsPerSecond = velocity;
    if (pushingIntoHardstop) {
      inputs.statorCurrentAmps = kStallCurrentAmps;
    } else if (velocity != 0.0) {
      inputs.statorCurrentAmps = kMovingCurrentAmps;
    } else {
      inputs.statorCurrentAmps = 0.0;
    }
  }

  @Override
  public void setPosition(double rotations) {
    usingMotionMagic = true;
    targetRotations = rotations;
  }

  @Override
  public void setHomingOutput(double volts) {
    usingMotionMagic = false;
    homingVolts = volts;
  }

  @Override
  public void setEncoderPosition(double rotations) {
    encoderOffsetRotations = truePositionRotations - rotations;
  }

  @Override
  public void stop() {
    usingMotionMagic = false;
    homingVolts = 0.0;
  }
}
