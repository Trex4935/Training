// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;


public class RobotContainer {
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * 0.1)
      .withRotationalDeadband(maxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.Velocity);
  private final Telemetry telemetry = new Telemetry(maxSpeed);
  private final Limelight limelight = new Limelight("limelight-bow", "limelight-intake");
  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {

    drivetrain.registerTelemetry(telemetry::telemeterize);

    if (drivetrain.isAutoBuilderConfigured()) {
      autoChooser = AutoBuilder.buildAutoChooser();
    } else {
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("Do Nothing", Commands.none());
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    limelight.setDefaultCommand(limelight.run(() -> {
      var state = drivetrain.getState();
      for (Limelight.Measurement measurement :
          limelight.getMeasurement(state.Pose, state.Speeds.omegaRadiansPerSecond)) {
        drivetrain.addVisionMeasurement(
            measurement.poseEstimate.pose,
            measurement.poseEstimate.timestampSeconds,
            measurement.standardDeviations);
      }
    }).ignoringDisable(true));

    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive
        .withVelocityX(-driverController.getLeftY() * maxSpeed)
        .withVelocityY(-driverController.getLeftX() * maxSpeed)
        .withRotationalRate(-driverController.getRightX() * maxAngularRate)));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
