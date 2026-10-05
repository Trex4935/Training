// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

/**
 * The hardware contract for the pivot. Pivot only talks to this interface, so it does not know or
 * care whether it is running on a real TalonFX (PivotIOReal) or in simulation (PivotIOSim).
 */
public interface PivotIO {
  /** Everything Pivot reads from the hardware, refreshed once per loop by updateInputs(). */
  public static class PivotIOInputs {
    public double positionRotations = 0.0;
    public double velocityRotationsPerSecond = 0.0;
    /** Stator current: the current through the motor windings. It spikes when the pivot stalls. */
    public double statorCurrentAmps = 0.0;
  }

  /** Reads the latest sensor values into inputs. */
  default void updateInputs(PivotIOInputs inputs) {}

  /** Moves the pivot to a position (mechanism rotations) using Motion Magic. */
  default void setPosition(double rotations) {}

  /** HOMING ONLY: drives the pivot slowly toward the hardstop at the given voltage. */
  default void setHomingOutput(double volts) {}

  /** Tells the encoder "you are now at this position". Used to zero at the hardstop. */
  default void setEncoderPosition(double rotations) {}

  /** Stops the motor. */
  default void stop() {}
}
