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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.commands.DriveToTagPose;
import frc.robot.commands.TagApproach;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;


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
      var bootstrap = limelight.getBootstrapPose(
          Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond),
          state.Speeds.omegaRadiansPerSecond);
      if (bootstrap != null) {
        drivetrain.resetPose(bootstrap);
        limelight.initializeFieldHeading("Stable " + limelight.getSeedSource());
        // Keep the driver reference fixed through global pose initialization.
        SmartDashboard.putString("Vision/HeadingChangeReason", "Global pose seeded from stable MegaTag1");
      }
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

    // Redefine only the driver's field-centric reference; the estimator pose stays global.
    driverController.leftBumper().onTrue(Commands.runOnce(() -> {
      var heading = drivetrain.getState().Pose.getRotation();
      drivetrain.setOperatorPerspectiveForward(heading);
      SmartDashboard.putString("Vision/HeadingChangeReason", "Driver forward calibrated");
      SmartDashboard.putString("Vision/ControlMode", "Manual");
    }));
    driverController.back().onTrue(Commands.runOnce(() -> {
      drivetrain.setOperatorPerspectiveForward(edu.wpi.first.math.geometry.Rotation2d.kZero);
      SmartDashboard.putString("Vision/HeadingChangeReason", "Standard blue perspective requested");
      SmartDashboard.putString("Vision/ControlMode", "Competition");
    }));

    PIDController tagPid = new PIDController(0.035, 0.0, 0.001);
    driverController.rightBumper().whileTrue(Commands.run(() -> {
      double yaw = Limelight.getForwardYawDegrees();
      double omega = 0;
      if (Double.isFinite(yaw)) {
        omega = Math.max(-1.5, Math.min(1.5, -tagPid.calculate(yaw, 0)));
        if (Math.abs(yaw) < 1.5) omega = 0;
      }
      drivetrain.setControl(drive.withVelocityX(-driverController.getLeftY() * maxSpeed)
          .withVelocityY(-driverController.getLeftX() * maxSpeed)
          .withRotationalRate(omega));
    }, drivetrain).beforeStarting(tagPid::reset));

    // Hallway example layout: tag 1 at (2, 1) m, facing away from the tag along its local +X.
    // Replace this with the measured hallway pose. Robot-front radius plus clearance is 0.55 m.
    Pose2d hallwayTag = new Pose2d(2.0, 1.0, Rotation2d.kZero);
    Pose2d approach = TagApproach.target(hallwayTag, 0.8, 0.42, 0.13);
    driverController.a().whileTrue(new DriveToTagPose(drivetrain, () -> approach, limelight::isDriveToTagValid));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
