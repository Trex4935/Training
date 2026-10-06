// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;


public class PivotIOReal implements PivotIO {


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


}
