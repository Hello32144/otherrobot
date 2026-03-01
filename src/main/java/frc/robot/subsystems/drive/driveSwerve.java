package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class driveSwerve extends SubsystemBase {
    private TalonFX m_Drive_Motor;
    private  TalonFX m_Steer_Motor;
    private CANcoder m_Encoder;
   private TalonFXConfiguration m_Drive_Motor_Configure;
   private TalonFXConfiguration m_Steer_Motor_Configure;
    private CANcoderConfiguration m_Encoder_Configure;
    private PIDController m_Drive_PID = new PIDController(0.3, 0, 0); //ADJUST VALUES LATER
    private PIDController m_Steer_PID = new PIDController(0.1, 0, 0); //ADJUST VALUES LATER
    public driveSwerve(int drive_ID, int steer_ID, int m_Encoder_Id){
    m_Drive_Motor = new TalonFX(drive_ID);
    m_Steer_Motor = new TalonFX(steer_ID);
    m_Encoder = new CANcoder(m_Encoder_Id);
    m_Drive_Motor_Configure = new TalonFXConfiguration();
    m_Steer_Motor_Configure = new TalonFXConfiguration();
    m_Encoder_Configure = new CANcoderConfiguration();
    
    m_Drive_Motor_Configure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_Drive_Motor_Configure.CurrentLimits.SupplyCurrentLimit = 30;
    m_Drive_Motor_Configure.Feedback.SensorToMechanismRatio = 0;//adjust later to real values
    
    m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimitEnable = true;
    m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimit = 10;
    m_Drive_Motor_Configure.CurrentLimits.StatorCurrentLimitEnable = true; 
    m_Drive_Motor_Configure.ClosedLoopGeneral.ContinuousWrap = true;

    m_Steer_Motor_Configure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_Steer_Motor_Configure.Feedback.RotorToSensorRatio =1;//adjust later to real values
    m_Steer_Motor_Configure.CurrentLimits.SupplyCurrentLimit=15;
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
    m_Drive_PID.enableContinuousInput(-180, 180);
    m_Steer_PID.enableContinuousInput(-180, 180);

}
public void getState(){
    drive.
}
public void setPosition(driveSwerve state){
    
}
public void periodic(){

}
}
