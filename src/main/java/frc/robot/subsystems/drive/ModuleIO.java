package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVelocityRps = 0.0;
        public double drivePositionRot = 0.0;
        public double steerAbsolutePositionRot = 0.0;
        public double driveAppliedVolts = 0.0;
        public double steerAppliedVolts = 0.0;
    }
}