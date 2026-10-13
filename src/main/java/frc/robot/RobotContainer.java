// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;

public class RobotContainer {
            public static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                       // second
                                                                                       // max angular velocity
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
        private final Telemetry logger = new Telemetry(MaxSpeed);
        private final ProfiledPIDController hubPID = new ProfiledPIDController(
                        5.0, 0.0, 0.0,
                        new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate * 2));
  private final Pivot pivot;

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    PivotIO pivotIO = RobotBase.isReal() ? new PivotIOReal() : new PivotIOSim();
    pivot = new Pivot(pivotIO);
    configureBindings();
  }

  private void configureBindings() {

   // RobotModeTriggers.autonomous()
     //   .or(RobotModeTriggers.teleop())
       // .onTrue(pivot.home().unless(pivot::isHomed));

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));
    driverController.a().onTrue(
        Commands.runOnce(() -> pivot.goToPositionDegrees(PivotState.DOWN.degrees), pivot));
    driverController.b().onTrue(
        Commands.runOnce(() -> pivot.goToPositionDegrees(PivotState.STOW.degrees), pivot));
    driverController.start().onTrue(pivot.home());
    SmartDashboard.putData(
        "Pivot/RESET ZERO AT HARDSTOP",
        Commands.runOnce(pivot::resetZeroAtHardstop, pivot).ignoringDisable(true));
    driverController.povUp().onTrue(pivot.agitate);

    SmartDashboard.putNumber("Pivot/Target Degrees Slider", PivotState.DOWN.degrees);
    driverController.x().whileTrue(Commands.run(
        () -> pivot.goToPositionDegrees(
            SmartDashboard.getNumber("Pivot/Target Degrees Slider", PivotState.DOWN.degrees)),
        pivot));
  }
}
