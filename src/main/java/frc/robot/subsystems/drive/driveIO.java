// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

private final Pigeon2 m_gyro = new Pigeon2(1);

private Pose2d m_Pose = new Pose2d();
private SwerveDriveOdometry m_Odometry;

private final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
    drivevalues.m_Left_Front, drivevalues.m_Right_Front,
    drivevalues.m_Left_Back, drivevalues.m_Right_Back);
  public driveIO(driveModuleIO front_left, driveModuleIO front_right, driveModuleIO back_right, driveModuleIO back_left) {
    this.front_left_io = front_left;
    this.front_right_io = front_right;
    this.back_left_io = back_left;
    this.back_right_io = back_right;
  
m_Odometry = new SwerveDriveOdometry(m_Kinematics, m_gyro.getRotation2d(),  new SwerveModulePosition[]{
      new SwerveModulePosition(front_left_inputs.drive_Difference[0] , front_left_inputs.drive_Direction[0]),
      new SwerveModulePosition(front_right_inputs.drive_Difference[0] , front_right_inputs.drive_Direction[0]),
      new SwerveModulePosition(back_left_inputs.drive_Difference[0] , back_left_inputs.drive_Direction[0]),
      new SwerveModulePosition(back_right_inputs.drive_Difference[0] , back_right_inputs.drive_Direction[0])
    });
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

m_Odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[]{
    new SwerveModulePosition(front_left_inputs.drive_Difference[0], front_left_inputs.drive_Direction[0]),
    new SwerveModulePosition(front_right_inputs.drive_Difference[0], front_right_inputs.drive_Direction[0]),
    new SwerveModulePosition(back_left_inputs.drive_Difference[0], back_left_inputs.drive_Direction[0]),
    new SwerveModulePosition(back_right_inputs.drive_Difference[0], back_right_inputs.drive_Direction[0])
});
Logger.recordOutput("Pose", m_Odometry.getPoseMeters());

  }
}

