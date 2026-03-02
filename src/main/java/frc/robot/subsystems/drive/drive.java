// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;



import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A simple swerveDrive system that has not been tested and needs values update
 * 
 * @author William Ding
 */
public class drive extends SubsystemBase {
  private final Pigeon2 m_gyro = new Pigeon2(0);// can change gyro later

  private  driveSwerve m_Left_Front_Module = new driveSwerve(6, 7, 8);
  private driveSwerve m_Left_Back_Module = new driveSwerve(3, 4, 5);
  private  driveSwerve m_Right_Front_Module = new driveSwerve(9, 10, 11);
  private driveSwerve m_Right_Back_Module = new driveSwerve(12, 13, 14);

  Pose2d m_Pose = new Pose2d();

  private final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(drivevalues.m_Left_Front,
      drivevalues.m_Left_Back, drivevalues.m_Right_Front, drivevalues.m_Right_Back);

  public drive() {// add reset gyro if we attach and add rotation2d on cahssis speeds
  }

  public void driveRobot(double x, double y, double Rotation) {
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, Rotation);
    SwerveModuleState[] neededPositions = m_Kinematics.toSwerveModuleStates(speeds);

    m_Left_Front_Module.setPosition(neededPositions[0]);
    m_Left_Back_Module.setPosition(neededPositions[1]);
    m_Right_Front_Module.setPosition(neededPositions[2]);
    m_Right_Back_Module.setPosition(neededPositions[3]);
  }

  // if we have a gyro then we can use this constructor
    SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(m_Kinematics, m_gyro.getRotation2d(),
      new SwerveModulePosition[] { m_Left_Front_Module.getPosition(),
          m_Left_Back_Module.getPosition(), m_Right_Front_Module.getPosition(), m_Right_Back_Module.getPosition() });

public Pose2d getPose(){
  return m_Odometry.getPoseMeters();
}

  @Override
  public void periodic() {
var m_gyro_angle = m_gyro.getRotation2d();
m_Odometry.update(m_gyro_angle,  new SwerveModulePosition[] { m_Left_Front_Module.getPosition(),
          m_Left_Back_Module.getPosition(), m_Right_Front_Module.getPosition(), m_Right_Back_Module.getPosition() });
  }
}
