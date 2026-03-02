package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
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

/**
 * A simple swerveDrive system that has not been tested and needs values update
 * 
 * @author William Ding
 */
public class driveSwerve extends SubsystemBase {
    private TalonFX m_Drive_Motor;
    private TalonFX m_Steer_Motor;
    private CANcoder m_Encoder;

    private TalonFXConfiguration m_Drive_Motor_Configure;
    private TalonFXConfiguration m_Steer_Motor_Configure;
    private CANcoderConfiguration m_Encoder_Configure;
    private PIDController m_Drive_PID = new PIDController(0.3, 0, 0); // ADJUST VALUES LATER
    private PIDController m_Steer_PID = new PIDController(0.1, 0, 0); // ADJUST VALUES LATER

    public driveSwerve(int drive_ID, int steer_ID, int m_Encoder_Id) {
        m_Drive_Motor = new TalonFX(drive_ID);
        m_Steer_Motor = new TalonFX(steer_ID);
        m_Encoder = new CANcoder(m_Encoder_Id);
        m_Drive_Motor_Configure = new TalonFXConfiguration();
        m_Steer_Motor_Configure = new TalonFXConfiguration();
        m_Encoder_Configure = new CANcoderConfiguration();

        m_Drive_Motor_Configure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_Drive_Motor_Configure.CurrentLimits.SupplyCurrentLimit = 30;
        m_Drive_Motor_Configure.Feedback.SensorToMechanismRatio = 0;// adjust later to real values

        m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimitEnable = true;
        m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimit = 10;
        m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimitEnable = true;
        m_Drive_Motor_Configure.ClosedLoopGeneral.ContinuousWrap = true;

        m_Steer_Motor_Configure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_Steer_Motor_Configure.Feedback.RotorToSensorRatio = 1;// adjust later to real values
        m_Steer_Motor_Configure.CurrentLimits.SupplyCurrentLimit = 15;
        m_Steer_Motor_Configure.CurrentLimits.SupplyCurrentLimitEnable = true;
        m_Steer_Motor_Configure.CurrentLimits.StatorCurrentLimit = 10;
        m_Steer_Motor_Configure.CurrentLimits.StatorCurrentLimitEnable = true;
        m_Steer_Motor_Configure.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        m_Steer_Motor_Configure.Feedback.FeedbackRemoteSensorID = m_Encoder_Id;
        m_Steer_Motor_Configure.ClosedLoopGeneral.ContinuousWrap = true;

        m_Encoder_Configure.MagnetSensor.MagnetOffset = 0;
        m_Encoder_Configure.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        m_Encoder.getConfigurator().apply(m_Encoder_Configure);
        m_Drive_Motor.getConfigurator().apply(m_Drive_Motor_Configure);
        m_Steer_Motor.getConfigurator().apply(m_Steer_Motor_Configure);
        m_Drive_PID.enableContinuousInput(-0.5, 0.5);
        m_Steer_PID.enableContinuousInput(-0.5, 0.5);

    }

    public SwerveModuleState getState() {
        double velocity = m_Drive_Motor.getVelocity().getValueAsDouble();
        Rotation2d angle = Rotation2d.fromRotations(m_Encoder.getAbsolutePosition().getValueAsDouble());
        SwerveModuleState state = new SwerveModuleState(velocity, angle);
        return state;
    }

    public SwerveModulePosition getPosition() {
        double distance = (m_Drive_Motor.getPosition().getValueAsDouble() / drivevalues.gear_ratio)
                * drivevalues.m_wheel_diameter.in(Meters) * Math.PI;
        Rotation2d angle = Rotation2d.fromRotations(m_Encoder.getAbsolutePosition().getValueAsDouble());
        SwerveModulePosition position = new SwerveModulePosition(distance, angle);
        return position;
    }

    public void setPosition(SwerveModuleState state) {
        state.optimize(getState().angle);
        double steer_speed = m_Steer_PID.calculate(m_Encoder.getAbsolutePosition().getValueAsDouble(),
                state.angle.getRotations());
        double drive_speed = m_Drive_PID.calculate(m_Drive_Motor.getVelocity().getValueAsDouble(),
                state.speedMetersPerSecond);

        m_Drive_Motor.set(drive_speed);
        m_Steer_Motor.set(steer_speed);
    }

    public void periodic() {

    }
}
