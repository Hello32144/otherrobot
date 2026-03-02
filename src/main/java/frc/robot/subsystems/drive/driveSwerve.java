package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSwerve extends SubsystemBase {
    
    // Uses the class generated from ModuleIO.java
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final String moduleName;

    private TalonFX m_Drive_Motor;
    private TalonFX m_Steer_Motor;
    private CANcoder m_Encoder;

    private PIDController m_Drive_PID = new PIDController(0.3, 0, 0); 
    private PIDController m_Steer_PID = new PIDController(0.1, 0, 0); 

    public DriveSwerve(int drive_ID, int steer_ID, int m_Encoder_Id) {
        this.moduleName = "Module" + drive_ID;
        
        m_Drive_Motor = new TalonFX(drive_ID);
        m_Steer_Motor = new TalonFX(steer_ID);
        m_Encoder = new CANcoder(m_Encoder_Id);

        configureHardware(m_Encoder_Id);
        m_Steer_PID.enableContinuousInput(-0.5, 0.5);
    }

    private void configureHardware(int encoderId) {
        var driveConfig = new TalonFXConfiguration();
        var steerConfig = new TalonFXConfiguration();
        var encoderConfig = new CANcoderConfiguration();

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        m_Encoder.getConfigurator().apply(encoderConfig);
        m_Drive_Motor.getConfigurator().apply(driveConfig);
        m_Steer_Motor.getConfigurator().apply(steerConfig);
    }

    public void updateInputs() {
        inputs.driveVelocityRps = m_Drive_Motor.getVelocity().getValueAsDouble();
        inputs.drivePositionRot = m_Drive_Motor.getPosition().getValueAsDouble();
        inputs.steerAbsolutePositionRot = m_Encoder.getAbsolutePosition().getValueAsDouble();
        inputs.driveAppliedVolts = m_Drive_Motor.getMotorVoltage().getValueAsDouble();
        inputs.steerAppliedVolts = m_Steer_Motor.getMotorVoltage().getValueAsDouble();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocityRps * (drivevalues.m_wheel_diameter.in(Meters) * Math.PI) / drivevalues.gear_ratio,
            Rotation2d.fromRotations(inputs.steerAbsolutePositionRot)
        );
    }

    public SwerveModulePosition getPosition() {
        double distance = (inputs.drivePositionRot / drivevalues.gear_ratio)
                * drivevalues.m_wheel_diameter.in(Meters) * Math.PI;
        return new SwerveModulePosition(distance, Rotation2d.fromRotations(inputs.steerAbsolutePositionRot));
    }

    public void setModuleState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, Rotation2d.fromRotations(inputs.steerAbsolutePositionRot));

        double steer_speed = m_Steer_PID.calculate(inputs.steerAbsolutePositionRot, optimizedState.angle.getRotations());
        double targetRps = (optimizedState.speedMetersPerSecond / (drivevalues.m_wheel_diameter.in(Meters) * Math.PI) * drivevalues.gear_ratio);
        double drive_speed = m_Drive_PID.calculate(inputs.driveVelocityRps, targetRps);

        m_Drive_Motor.set(drive_speed);
        m_Steer_Motor.set(steer_speed);

        Logger.recordOutput("Drive/Setpoints/" + moduleName, optimizedState);
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Drive/" + moduleName, inputs);
        Logger.recordOutput("Drive/ActualStates/" + moduleName, getState());
    }
}