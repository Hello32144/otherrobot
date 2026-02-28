package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class driveSwerve {
    private TalonFX m_Drive_Motor;
    private  TalonFX m_Steer_Motor;
   private TalonFXConfiguration m_Drive_Motor_Configure = new TalonFXConfiguration();
   private TalonFXConfiguration m_Steer_Motor_Configure = new TalonFXConfiguration();

    public driveSwerve(int drive_ID, int steer_ID){
    m_Drive_Motor = new TalonFX(drive_ID);
    m_Steer_Motor = new TalonFX(steer_ID);
    m_Drive_Motor_Configure = new TalonFXConfiguration();
    m_Steer_Motor_Configure = new TalonFXConfiguration();
    m_Drive_Motor_Configure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_Drive_Motor_Configure.MotorOutput.
    m_Drive_Motor.getConfigurator().apply(m_Drive_Motor_Configure);
    m_Steer_Motor.getConfigurator().apply(m_Steer_Motor_Configure); 
    



}
}
