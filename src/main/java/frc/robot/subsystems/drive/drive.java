// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class drive extends SubsystemBase {
private final Pigeon2 m_gyro = new Pigeon2(0);//can change gyro later

private static driveSwerve m_Left_Front_Module= new driveSwerve(3, 4, 5);
private static driveSwerve m_Left_Back_Module = new driveSwerve(6, 7, 8);
private static driveSwerve m_Right_Front_Module = new driveSwerve(9, 10, 11);
private static driveSwerve m_Right_Back_Module = new driveSwerve(12, 13, 14);

private final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(drivevalues.m_Left_Front, drivevalues.m_Left_Back,drivevalues.m_Right_Front,drivevalues.m_Right_Back);

public drive() {

  }

  @Override
  public void periodic() {

    
  }
}
