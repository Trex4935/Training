// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotState;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;

public class RobotContainer {
  private final Pivot pivot;

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    PivotIO pivotIO = RobotBase.isReal() ? new PivotIOReal() : new PivotIOSim();
    pivot = new Pivot(pivotIO);
    configureBindings();
  }

  private void configureBindings() {

    RobotModeTriggers.autonomous()
        .or(RobotModeTriggers.teleop())
        .onTrue(pivot.home().unless(pivot::isHomed));

    driverController.a().onTrue(
        Commands.runOnce(() -> pivot.goToPositionDegrees(PivotState.DOWN.degrees), pivot));
    driverController.b().onTrue(
        Commands.runOnce(() -> pivot.goToPositionDegrees(PivotState.STOW.degrees), pivot));
    driverController.start().onTrue(pivot.home());

    SmartDashboard.putData(
        "Pivot/RESET ZERO AT HARDSTOP",
        Commands.runOnce(pivot::resetZeroAtHardstop, pivot).ignoringDisable(true));
  }
}
