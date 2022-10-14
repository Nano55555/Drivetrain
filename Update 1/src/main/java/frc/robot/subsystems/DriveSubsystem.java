// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
public class DriveSubsystem extends SubsystemBase {
  private static final WPI_TalonFX leftFrontMotor= RobotMap.leftFrontDrivePort;
  private static final WPI_TalonFX rightFrontMotor= RobotMap.rightFrontDrivePort;
  private static final WPI_TalonFX leftBackMotor= RobotMap.leftBackDrivePort;
  private static final WPI_TalonFX rightBackMotor= RobotMap.rightBackDrivePort;

  private static final double In_To_M=.0254;
  private static final int Motor_Encoder_Codes_Per_Rev=2048;
  private static final double Diameter_Inches=5.0;
  private static final double Wheel_Diameter= Diameter_Inches * In_To_M;
  private static final double Wheel_Circumference= Wheel_Diameter * Math.PI;
  private static final double Gear_Ratio=12.75;
  private static final double Ticks_Per_Meter= ( Motor_Encoder_Codes_Per_Rev * Gear_Ratio)/(Wheel_Circumference);
  private static final double Meters_Per_Ticks= 1/Ticks_Per_Meter;
  /** Creates a new DriveSubsystem. */
  public void setModePercentVoltage(){
    leftFrontMotor.set(ControlMode.PercentOutput, 0);
    rightFrontMotor.set(ControlMode.PercentOutput, 0);
    leftBackMotor.set(ControlMode.PercentOutput, 0);
    rightBackMotor.set(ControlMode.PercentOutput, 0);
  }
  public DriveSubsystem() {
    leftFrontMotor.set(ControlMode.Follower, leftBackMotor.getDeviceID());
    rightFrontMotor.set(ControlMode.Follower, rightBackMotor.getDeviceID()); 
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,1);
    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftFrontMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms);
    leftFrontMotor.configVelocityMeasurementWindow(16);
    leftFrontMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,5,10);

    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    
    leftFrontMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    leftBackMotor.setInverted(false);
    rightFrontMotor.setInverted(true);
    resetEncoders();
  }
  public void resetEncoders() {
    leftBackMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
  }
  public void drive(double throttle, double rotate) {
    leftFrontMotor.set(throttle +rotate);
    rightFrontMotor.set(throttle - rotate);
    leftBackMotor.set(throttle + rotate);
    rightBackMotor.set(throttle - rotate);
  }
  public double getRightBackEncoderPosition(){
    return rightBackMotor.getSelectedSensorPosition();
  }
  public double getLeftBackEncoderPosition(){
    return leftBackMotor.getSelectedSensorPosition();
  }
  public double distanceTravelledinTick(){
    return (getLeftBackEncoderPosition() + getRightBackEncoderPosition())/2;
  }
  public double getLeftBackEncoderPositionVelocityMetersPerSecond()
  {
    double leftVelocityMPS = (leftBackMotor.getSelectedSensorPosition()*10);
    leftVelocityMPS = leftVelocityMPS * Meters_Per_Ticks;
    return (leftVelocityMPS);
  }
  public double leftDistanceTravelledInMeters(){
    double left_dist=getRightBackEncoderPosition() * Meters_Per_Ticks;
    return left_dist;
  }
  public void stop() {
    drive(0,0);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
