// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;

public interface driveModuleIO {
  @AutoLog
  public static class driveLogger {
    public double m_Drive_Motor_IO_Meters_Per_Second = 0;
    public double m_Drive_Motor_IO_Volts = 0;
    public double m_Drive_Motor_IO_Amps = 0;
    public double m_Drive_Motor_IO_Distance = 0;

    public double m_Steer_Motor_IO_Volts = 0;
    public double m_Steer_Motor_IO_Amps = 0;
    public Rotation2d m_Steer_Motor_IO_Positon = new Rotation2d();
    public double m_Steer_Motor_IO_Rotations_Per_Second = 0;

    public double[] time = new double[]{};
    public double[] drive_Difference = new double[]{0};
    public Rotation2d[] drive_Direction = new Rotation2d[]{new Rotation2d()};

    public double[] pose2d = new double[]{};

  }

  public default void updateInputs(driveLogger inputs) {
  }

  public default void setDriveVoltage(double volts) {
  }

  public default void setSteerVoltage(double volts) {

  }

  public void setDriveVoltage(SwerveModuleState swerveModuleState, ChassisSpeeds speeds);
}
