// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /**
   * Pivot constants. This is the SAME physical mechanism as the 2026 robot's intake pivot, so every
   * value below is copied from the 2026 code (Constants.java, IntakeIOReal.java, Intake.java).
   * Do not retune them here.
   *
   * <p>Units: positions, velocities and accelerations are in PIVOT rotations (1.0 = one full turn
   * of the arm, 0.25 = 90 degrees). 0.0 is the hardstop the pivot homes against. Moving away from
   * the hardstop is NEGATIVE (2026 stows at -245 degrees).
   */
  public static class PivotConstants {
    // Hardware (2026: Constants.intakePivot, IntakeIOReal.pivotConfig())
    public static final int kMotorCanId = 2;
    /** false = CounterClockwise_Positive, which is what 2026 uses. */
    public static final boolean kMotorInverted = false;
    /** Motor rotations per one pivot rotation (2026: kIntakePivotReduction). */
    public static final double kGearRatio = 17.5;
    public static final double kStatorCurrentLimitAmps = 120.0;
    public static final double kSupplyCurrentLimitAmps = 70.0;

    // Safe travel range. Every target is clamped into this range.
    // 2026 has no software position limit, so this range is bounded by the positions 2026
    // already uses: STOW (-245 deg) and the hardstop (0 deg).
    public static final double kMinPositionRotations = Units.degreesToRotations(-245.0); // TODO: confirm acceptable as the lower limit
    public static final double kMaxPositionRotations = 0.0; // the hardstop

    // Motion Magic profile (2026: cruise = 6000 RPM, accel = 6000 RPM / reduction per second)
    public static final double kCruiseVelocity = 6000.0 / 60.0; // 100 rotations/sec
    public static final double kAcceleration = kCruiseVelocity / kGearRatio; // ~5.71 rotations/sec^2

    // Slot0 gains (2026: kP 70, kD 0, kV 12 / 100 rps; kI, kS, kA, kG not set, so 0)
    public static final double kP = 70.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 12.0 / 100.0; // 0.12 volts per rotation/sec

    // Homing (2026: Intake.java homing constants)
    /** Volts applied while seeking the hardstop. +3 V is the direction proven to home this pivot. */
    public static final double kHomingVolts = 3.0;
    /** Stator current that means "we are pushing on something". */
    public static final double kHomingCurrentThresholdAmps = 65.0;
    /** Below this speed we consider the pivot stopped (2026: 30 deg/s). */
    public static final double kHomingVelocityThreshold = Units.degreesToRotations(30.0);
    /** Ignore the hardstop check this long after homing starts: startup current looks like a stall. */
    public static final double kHomingStartupGraceSeconds = 0.15;
    /** The hardstop condition must stay true this long before we believe it. */
    public static final double kHardstopDebounceSeconds = 0.06;
    /** Give up homing after this long so the motor can never run forever. */
    public static final double kHomingTimeoutSeconds = 1.0;
  }
}
