// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.PivotConstants;

/** PivotIO for the real TalonFX. This is the only pivot file that touches CTRE code. */
public class PivotIOReal implements PivotIO {
  private final TalonFX motor = new TalonFX(PivotConstants.kMotorCanId);

  /** Normal movement: Motion Magic to a position. */
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  /** Homing only: a small fixed voltage toward the hardstop. */
  private final VoltageOut homingRequest = new VoltageOut(0);

  public PivotIOReal() {
    TalonFXConfiguration config = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    PivotConstants.kMotorInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive))
        .withFeedback(
            // With this ratio set, the TalonFX reports position in PIVOT rotations, not motor
            // rotations, so every number in PivotConstants is in pivot rotations.
            new FeedbackConfigs()
                .withSensorToMechanismRatio(PivotConstants.kGearRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(PivotConstants.kStatorCurrentLimitAmps)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(PivotConstants.kSupplyCurrentLimitAmps)
                .withSupplyCurrentLimitEnable(true))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(PivotConstants.kCruiseVelocity)
                .withMotionMagicAcceleration(PivotConstants.kAcceleration))
        .withSlot0(
            new Slot0Configs()
                .withKP(PivotConstants.kP)
                .withKI(PivotConstants.kI)
                .withKD(PivotConstants.kD)
                .withKV(PivotConstants.kV));
    motor.getConfigurator().apply(config);

    // Current is only sent 4 times per second by default. Homing checks it every 20 ms loop,
    // so ask for fresh current and velocity at 100 Hz (same as our 2026 intake pivot).
    BaseStatusSignal.setUpdateFrequencyForAll(
        100, motor.getStatorCurrent(), motor.getSupplyCurrent(), motor.getVelocity());
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRotations = motor.getPosition().getValueAsDouble();
    inputs.velocityRotationsPerSecond = motor.getVelocity().getValueAsDouble();
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setPosition(double rotations) {
    motor.setControl(motionMagicRequest.withPosition(rotations));
  }

  @Override
  public void setHomingOutput(double volts) {
    motor.setControl(homingRequest.withOutput(volts));
  }

  @Override
  public void setEncoderPosition(double rotations) {
    motor.setPosition(rotations);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
