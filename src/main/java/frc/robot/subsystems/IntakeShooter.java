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
    private SparkMax indexer = new SparkMax(1, MotorType.kBrushless);
    private SparkMax intakeShootMotor = new SparkMax(2, MotorType.kBrushless);

    private SparkMaxConfig indexConfig = new SparkMaxConfig();
    private SparkMaxConfig motorConfig = new SparkMaxConfig();

    public boolean indexerRunning = false;
    public boolean motorRunning = false;
     
    /** Creates a new IntakeShooter. */
    public IntakeShooter() {
        indexer.configure(indexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        intakeShootMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void intakeShoot(double speed) {
        motorRunning = true;
        intakeShootMotor.set(speed);
    }

    public void stopIntakeShoot() {
        motorRunning = false;
        intakeShootMotor.set(0);
    }

    public double intakeSpeed() {
        return intakeShootMotor.getEncoder().getVelocity();
    }

    public void index(double speed) {
        indexerRunning = true;
        indexer.set(speed);
    }

    public void stopIndex() {
        indexerRunning = false;
        indexer.set(0);
    }

    public double indexSpeed() {
        return indexer.getEncoder().getVelocity();
    }
}
