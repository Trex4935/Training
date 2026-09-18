// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final TalonFX motor = new TalonFX(3);
  public ExampleSubsystem() {
  }


  public void move() {
    motor.set(0.5);
  }

  public void stop() {
    motor.stopMotor();
  }

  public Command moveCMD(){
    return Commands.startEnd(() -> move(), ()-> stop());
  }
}
