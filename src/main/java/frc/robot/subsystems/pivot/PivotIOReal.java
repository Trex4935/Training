// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import java.security.KeyStore.PrivateKeyEntry;

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

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.PivotConstants;

public class PivotIOReal implements PivotIO {
    private final TalonFX motor = new TalonFX(PivotConstants.kMotorCanId);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

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
                .withMotionMagicCruiseVelocity(
                    Units.degreesToRotations(PivotConstants.kCruiseVelocityDegreesPerSecond))
                .withMotionMagicAcceleration(
                    Units.degreesToRotations(PivotConstants.kAccelerationDegreesPerSecondSquared)))
        .withSlot0(
            new Slot0Configs()
                .withKP(PivotConstants.kP)
                .withKI(PivotConstants.kI)
                .withKD(PivotConstants.kD)
                .withKV(PivotConstants.kV));
    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, motor.getStatorCurrent(), motor.getSupplyCurrent(), motor.getVelocity());
  }

  public void updateInputs(PivotIOInputs inputs) {
    inputs.postionDegrees = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    inputs.velocityDegreesPerSecond = Units.rotationsToDegrees(motor.getVelocity().getValueAsDouble());
    inputs.statorCurrentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  public void setPositionDegrees(double degrees){
    motor.setControl(motionMagicRequest.withPosition(Units.degreesToRotations(degrees)));
  }
  public void setHomingOutput(double volts){
    motor.setControl(homingRequest.withOutput(volts));
  }

  public void setEncoderPositionDegrees(double degrees){
    motor.setPosition(Units.degreesToRotations(degrees));
  }

  public void stop(){
    motor.stopMotor();
  }
}
