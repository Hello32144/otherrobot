// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
    private SparkMax m_indexer = new SparkMax(1, MotorType.kBrushless);
    private SparkMax m_intakeShootMotor = new SparkMax(2, MotorType.kBrushless);

    private SparkMaxConfig m_indexConfig = new SparkMaxConfig();
    private SparkMaxConfig m_motorConfig = new SparkMaxConfig();

    public boolean m_indexerRunning = false;
    public boolean motorRunning = false;
     
    /** Creates a new IntakeShooter. */
    public IntakeShooter() {
        m_indexer.configure(m_indexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_intakeShootMotor.configure(m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void intakeShoot(double speed) {
        motorRunning = true;
        m_intakeShootMotor.set(speed);
    }

    public void stopIntakeShoot() {
        motorRunning = false;
        m_intakeShootMotor.set(0);
    }

    public double intakeSpeed() {
        return m_intakeShootMotor.getEncoder().getVelocity();
    }

    public void index(double speed) {
        m_indexerRunning = true;
        m_indexer.set(speed);
    }

    public void stopIndex() {
        m_indexerRunning = false;
        m_indexer.set(0);
    }

    public double indexSpeed() {
        return m_indexer.getEncoder().getVelocity();
    }
}