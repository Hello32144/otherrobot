// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import java.security.Timestamp;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * @author willidng
 */
public class driveModuleIOHardware implements driveModuleIO {
  private TalonFX m_Drive_Motor;
  private SparkMax m_Steer_Motor;
  private CANcoder m_Encoder;

  private TalonFXConfiguration m_Drive_Motor_Configure;
  private SparkMaxConfig m_Steer_Motor_Configure;
  private CANcoderConfiguration m_Encoder_Configure;
  private PIDController m_Drive_PID = new PIDController(0.3, 0, 0); // ADJUST VALUES LATER
  private PIDController m_Steer_PID = new PIDController(0.1, 0, 0); // ADJUST VALUES LATER

  public driveModuleIOHardware(int drive_ID, int steer_ID, int m_Encoder_Id) {

    m_Drive_Motor = new TalonFX(drive_ID);
    m_Steer_Motor = new SparkMax(steer_ID, MotorType.kBrushless);
    m_Encoder = new CANcoder(m_Encoder_Id);
    m_Drive_Motor_Configure = new TalonFXConfiguration();
    m_Steer_Motor_Configure = new SparkMaxConfig();
    m_Encoder_Configure = new CANcoderConfiguration();

    m_Drive_Motor_Configure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_Drive_Motor_Configure.Feedback.SensorToMechanismRatio = 0;// adjust later to real values
    m_Drive_Motor_Configure.CurrentLimits.SupplyCurrentLimit = 30;
    m_Drive_Motor_Configure.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimit = 10;
    m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimitEnable = true;

    m_Steer_Motor_Configure.smartCurrentLimit(15);
    m_Steer_Motor_Configure.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(-0.5, 0.5);
    m_Encoder.getConfigurator().apply(m_Encoder_Configure);
    m_Drive_Motor.getConfigurator().apply(m_Drive_Motor_Configure);
    m_Steer_Motor.configure(m_Steer_Motor_Configure, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

  }

  @Override
  public void updateInputs(driveLogger inputs) {
    inputs.m_Drive_Motor_IO_Amps = m_Drive_Motor.getSupplyCurrent().getValueAsDouble();
    inputs.m_Drive_Motor_IO_Volts = m_Drive_Motor.getMotorVoltage().getValueAsDouble();
    inputs.m_Drive_Motor_IO_Distance = ((m_Drive_Motor.getPosition().getValueAsDouble() / drivevalues.gear_ratio)
        * drivevalues.m_wheel_diameter * Math.PI);
    inputs.m_Drive_Motor_IO_Meters_Per_Second = ((m_Drive_Motor.getVelocity().getValueAsDouble()
        / drivevalues.gear_ratio) * drivevalues.m_wheel_diameter * Math.PI);

    inputs.m_Steer_Motor_IO_Amps = m_Steer_Motor.getOutputCurrent();
    inputs.m_Steer_Motor_IO_Volts = m_Steer_Motor.getBusVoltage() * m_Steer_Motor.getAppliedOutput();
    inputs.m_Steer_Motor_IO_Rotations_Per_Second = (m_Steer_Motor.getEncoder().getVelocity()) / 60;
    inputs.m_Steer_Motor_IO_Positon = Rotation2d.fromRotations(m_Encoder.getAbsolutePosition().getValueAsDouble());

    inputs.time = new double[] { Logger.getTimestamp() };
    inputs.drive_Difference = new double[] { (m_Drive_Motor.getPosition().getValueAsDouble() / drivevalues.gear_ratio)
        * drivevalues.m_wheel_diameter* Math.PI
    };
    inputs.drive_Direction = new Rotation2d[] {
        Rotation2d.fromRotations(m_Encoder.getAbsolutePosition().getValueAsDouble())
    };
  }

  @Override
  public void setDriveVoltage(SwerveModuleState swerveModuleState, ChassisSpeeds speeds) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveVoltage'");
  }
}
