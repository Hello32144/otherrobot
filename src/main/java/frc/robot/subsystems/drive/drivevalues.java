package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;

public final class drivevalues {
public static final int m_Left_Back_Id= 3;
public static final int m_Left_Steer_Id=4;
public static final int m_Left_Back_Encoder = 5;

public static final int m_Left_Front_Id= 6;
public static final int m_Left_Front_Steer_Id=7;
public static final int m_Left_Front_Encoder = 8;

public static final int m_Right_Front_Id= 9;
public static final int m_Right_Front_Steer_Id=10;
public static final int m_Right_Front_Encoder = 11;

public static final int m_Right_Back_Id= 12;
public static final int m_Right_Back_Steer_Id=13;
public static final int m_Right_Back_Encoder = 14;

public static Translation2d m_Left_Nack = new Translation2d(1,1);
public static Translation2d m_Left_Front = new Translation2d(1,1);

public static Translation2d m_Right_Back = new Translation2d(1,1);
public static Translation2d m_Right_Front = new Translation2d(1,1);

}
