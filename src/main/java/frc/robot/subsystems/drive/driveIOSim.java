// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;
/**
 * @author willidng
 */
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveIOSim implements driveModuleIO {
    private final DCMotorSim drive_Sim;
    private final DCMotorSim steer_Sim;
    private double m_Drive_Volts = 0;
    private double m_Steer_Volts =0;
    private double m_target_angle =0;
    private double m_target_speed =0;
    private final PIDController m_Steer_PID = new PIDController(5.0, 0, 0);
    private final PIDController m_Drive_PID = new PIDController(0.5, 0, 0);

    public driveIOSim() {
        drive_Sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.1, 0.01), DCMotor.getKrakenX60(1));
        steer_Sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(0.1, 0.01), DCMotor.getNEO(1));
        m_Steer_PID.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void updateInputs(driveLogger inputs) {

        m_Drive_Volts = m_Drive_PID.calculate(drive_Sim.getAngularVelocityRPM()/60, m_target_speed );
        m_Steer_Volts =m_Steer_PID.calculate(steer_Sim.getAngularPositionRotations(), m_target_angle);
        drive_Sim.setInputVoltage(MathUtil.clamp(m_Drive_Volts, -12.0, 12.0));
        steer_Sim.setInputVoltage(MathUtil.clamp(m_Steer_Volts, -12.0, 12.0));
        drive_Sim.update(0.02);
        steer_Sim.update(0.02);

        inputs.m_Drive_Motor_IO_Amps = Math.abs(drive_Sim.getCurrentDrawAmps());
        inputs.m_Drive_Motor_IO_Volts = m_Drive_Volts;
        inputs.m_Drive_Motor_IO_Distance = ((drive_Sim.getAngularPositionRotations() / drivevalues.gear_ratio)
                * drivevalues.m_wheel_diameter * Math.PI);
        inputs.m_Drive_Motor_IO_Meters_Per_Second = (drive_Sim.getAngularVelocityRPM() * drivevalues.m_wheel_diameter
                * Math.PI) / 60 * drivevalues.gear_ratio;

        inputs.m_Steer_Motor_IO_Amps = Math.abs(steer_Sim.getCurrentDrawAmps());
        inputs.m_Steer_Motor_IO_Volts = m_Steer_Volts;
        inputs.m_Steer_Motor_IO_Rotations_Per_Second = steer_Sim.getAngularVelocityRPM() / 60;
        inputs.m_Steer_Motor_IO_Positon = Rotation2d.fromRotations((steer_Sim.getAngularPositionRotations()));

        inputs.time = new double[] { Logger.getTimestamp() };
        inputs.drive_Difference = new double[] { (drive_Sim.getAngularPositionRotations() / drivevalues.gear_ratio)
                * drivevalues.m_wheel_diameter * Math.PI
        };
        inputs.drive_Direction = new Rotation2d[] {
                Rotation2d.fromRotations(steer_Sim.getAngularPositionRotations()) };

    }

    @Override
    public void setDriveVoltage(double volts) {
        m_Drive_Volts = volts / (drivevalues.m_wheel_diameter * Math.PI) * drivevalues.gear_ratio;
    }

    @Override
    public void setSteerVoltage(double volts) {
        m_target_angle = volts;
    }


}
