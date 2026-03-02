package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Drive extends SubsystemBase {
  // Uses the class generated from DriveIO.java
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final Pigeon2 m_gyro = new Pigeon2(0);

  private final DriveSwerve m_Left_Front_Module = new DriveSwerve(6, 7, 8);
  private final DriveSwerve m_Left_Back_Module = new DriveSwerve(3, 4, 5);
  private final DriveSwerve m_Right_Front_Module = new DriveSwerve(9, 10, 11);
  private final DriveSwerve m_Right_Back_Module = new DriveSwerve(12, 13, 14);

  private final SwerveDriveKinematics m_Kinematics = new SwerveDriveKinematics(
      drivevalues.m_Left_Front, drivevalues.m_Left_Back, 
      drivevalues.m_Right_Front, drivevalues.m_Right_Back);

  private final SwerveDriveOdometry m_Odometry;

  public Drive() {
    m_Odometry = new SwerveDriveOdometry(
      m_Kinematics, 
      m_gyro.getRotation2d(),
      getModulePositions()
    );
  }

  private void updateInputs() {
    inputs.gyroYawRad = m_gyro.getRotation2d().getRadians();
    inputs.modulePositions = getModulePositions();
    inputs.moduleStates = new SwerveModuleState[] {
      m_Left_Front_Module.getState(),
      m_Left_Back_Module.getState(),
      m_Right_Front_Module.getState(),
      m_Right_Back_Module.getState()
    };
  }

  public void driveRobot(double x, double y, double rotation) {
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, rotation);
    SwerveModuleState[] states = m_Kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.5);

    m_Left_Front_Module.setModuleState(states[0]);
    m_Left_Back_Module.setModuleState(states[1]);
    m_Right_Front_Module.setModuleState(states[2]);
    m_Right_Back_Module.setModuleState(states[3]);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_Left_Front_Module.getPosition(),
      m_Left_Back_Module.getPosition(),
      m_Right_Front_Module.getPosition(),
      m_Right_Back_Module.getPosition()
    };
  }

  public Pose2d getPose() {
    return m_Odometry.getPoseMeters();
  }

  @Override
public void periodic() {
    updateInputs(); 

    Logger.processInputs("Drive", inputs);


    m_Odometry.update(Rotation2d.fromRadians(inputs.gyroYawRad), inputs.modulePositions);

    Logger.recordOutput("Drive/Pose", m_Odometry.getPoseMeters());
    Logger.recordOutput("Drive/ActualStates", inputs.moduleStates); 
}
}