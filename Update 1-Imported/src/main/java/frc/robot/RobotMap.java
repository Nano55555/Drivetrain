package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class RobotMap {
    
    public static final int Left_front_drive_port=3;
    public static final int Right_front_drive_port=4;
    public static final int Left_back_drive_port=1;
    public static final int Right_back_drive_port=2;

    public static final int Intake_motor_port=5;

    public static WPI_TalonFX leftFrontDrivePort= new WPI_TalonFX(Left_front_drive_port);
    public static WPI_TalonFX rightFrontDrivePort= new WPI_TalonFX(Right_front_drive_port);
    public static WPI_TalonFX leftBackDrivePort= new WPI_TalonFX(Left_back_drive_port);
    public static WPI_TalonFX rightBackDrivePort= new WPI_TalonFX(Right_back_drive_port);
    public static WPI_TalonFX inTakeMotor = new WPI_TalonFX(Intake_motor_port);
}
