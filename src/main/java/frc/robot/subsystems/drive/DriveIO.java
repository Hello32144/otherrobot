package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double gyroYawRad = 0.0;
        public SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(), new SwerveModulePosition(), 
            new SwerveModulePosition(), new SwerveModulePosition()
        };
        public SwerveModuleState[] moduleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(), 
            new SwerveModuleState(), new SwerveModuleState()
        };
    }
}