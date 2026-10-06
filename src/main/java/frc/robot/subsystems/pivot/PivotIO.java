// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

// TODO (Implementation team): declare the PivotIO interface.
//  - A PivotIOInputs class: positionDegrees, velocityDegreesPerSecond, statorCurrentAmps
//  - default methods: updateInputs, setPositionDegrees, setHomingOutput,
//    setEncoderPositionDegrees, stop
public interface PivotIO {

    public static class PivotIOInputs {
        public double postionDegrees = 0.0;
        public double velocityDegreesPerSecond = 0.0;
        public double statorCurrentAmps = 0.0;
    }

    default void updateInputs(PivotIOInputs inputs) {
    }

    default void setPositionDegrees(double degrees) {
    }

    default void setHomingOutput(double volts) {
    }

    default void setEncoderPositionDegrees(double degrees) {
    }

    default void stop() {
    }
}
