// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.driveModuleIO.driveLogger;
public class driveIO extends SubsystemBase {
private driveModuleIO front_left_io;
private driveModuleIO front_right_io;
private driveModuleIO back_left_io;
private driveModuleIO back_right_io;

private driveLoggerAutoLogged front_left_inputs = new driveLoggerAutoLogged();
private driveLoggerAutoLogged front_right_inputs = new driveLoggerAutoLogged();
private driveLoggerAutoLogged back_right_inputs = new driveLoggerAutoLogged();
private driveLoggerAutoLogged back_left_inputs = new driveLoggerAutoLogged();


  public driveIO(driveModuleIO front_left, driveModuleIO front_right, driveModuleIO back_right, driveModuleIO back_left) {
    this.front_left_io = front_left;
    this.front_right_io = front_right;
    this.back_left_io = back_left;
    this.back_right_io = back_right;
  }

  @Override
  public void periodic() {
    front_left_io.updateInputs(front_left_inputs);
    Logger.processInputs("Front_Left_Moule", front_left_inputs);

    front_right_io.updateInputs(front_right_inputs);
    Logger.processInputs("Front_Right_Module", front_right_inputs);

    back_left_io.updateInputs(back_left_inputs);
    Logger.processInputs("Back_Left_Module", back_left_inputs);

    back_right_io.updateInputs(back_right_inputs);
    Logger.processInputs("Back_Rights", back_right_inputs);
  }
}

