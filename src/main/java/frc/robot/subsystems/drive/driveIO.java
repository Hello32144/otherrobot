// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.driveModuleIO.driveLogger;

public class driveIO extends SubsystemBase {
private driveModuleIO io;
private driveLoggerAutoLogged inputs = new driveLoggerAutoLogged();
private driveLogger[] modules = new driveLogger[]{
}
  public driveIO(driveModuleIO io) {
    this.io =io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
