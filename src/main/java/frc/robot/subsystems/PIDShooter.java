// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class PIDShooter extends SubsystemBase {
  /** Creates a new PIDShooter. */
  CANSparkMax lowShooter = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax topShooter = new CANSparkMax(6, MotorType.kBrushless);
  SparkPIDController highPidController = topShooter.getPIDController();
  SparkPIDController lowPidController = lowShooter.getPIDController();

  double highShooterSpeed = 0;
  double lowShooterSpeed = 0;

  public PIDShooter() {
    SmartDashboard.putNumber("lowShooter RPM", 1000);
    SmartDashboard.putNumber("highShooter RPM", 1000);
    SmartDashboard.putNumber("low P", 0);
    SmartDashboard.putNumber("low I", 0);
    SmartDashboard.putNumber("low D", 0);
    SmartDashboard.putNumber("high P", 0);
    SmartDashboard.putNumber("high I", 0);
    SmartDashboard.putNumber("high D", 0);
    topShooter.setInverted(true);
    topShooter.setSmartCurrentLimit(RobotConstants.shooterMotorCurrentLimit);
    lowShooter.setSmartCurrentLimit(RobotConstants.shooterMotorCurrentLimit);
  }
  /*
  public void on(){
    lowPidController.setReference(SmartDashboard.getNumber("lowShooter RPM", 0), ControlType.kVelocity);
    highPidController.setReference(SmartDashboard.getNumber("highShooter RPM", 0), ControlType.kVelocity);
  }
*/
  public void off(){
    lowPidController.setReference(0, ControlType.kVelocity);
    highPidController.setReference(0, ControlType.kVelocity);
  }
  @Override
  public void periodic() {
   if (SmartDashboard.getNumber("low P", 0) != lowPidController.getP()){
    lowPidController.setP(SmartDashboard.getNumber("low P", 0));
   }
   if (SmartDashboard.getNumber("low I", 0) != lowPidController.getI()){
    lowPidController.setI(SmartDashboard.getNumber("low I", 0));
   }
   if (SmartDashboard.getNumber("low D", 0) != lowPidController.getD()){
    lowPidController.setD(SmartDashboard.getNumber("low D", 0));
   }
   if (SmartDashboard.getNumber("high P", 0) != highPidController.getP()){
    highPidController.setP(SmartDashboard.getNumber("high P", 0));
   }
   if (SmartDashboard.getNumber("high I", 0) != highPidController.getI()){
    highPidController.setI(SmartDashboard.getNumber("high I", 0));
   }
   if (SmartDashboard.getNumber("high D", 0) != highPidController.getD()){
    highPidController.setD(SmartDashboard.getNumber("high D", 0));
   }
  }


  public boolean isReady() {
    // return true if shooter is at speed
    
    // velocity 
    boolean isTopSpeedReady =  (Math.abs(topShooter.getEncoder().getVelocity() - highShooterSpeed)/highShooterSpeed) < .05;
    boolean isLowSpeedReady =  (Math.abs(lowShooter.getEncoder().getVelocity() - lowShooterSpeed)/lowShooterSpeed) < .05;
    return isLowSpeedReady && isTopSpeedReady;
  }

  public void prepareForSpeaker() {
    // go the right speed to shoot into speaker
    highShooterSpeed = SmartDashboard.getNumber( "high shooter rpm for speaker", Constants.RobotConstants.highRpmSpeaker);
    highPidController.setReference(highShooterSpeed, ControlType.kVelocity);
    lowShooterSpeed = SmartDashboard.getNumber( "low shooter rpm for speaker", Constants.RobotConstants.lowRpmSpeaker);
    lowPidController.setReference(lowShooterSpeed, ControlType.kVelocity);
  }

  public void prepareForAmp() {
    highShooterSpeed = SmartDashboard.getNumber( "high shooter rpm for Amp", Constants.RobotConstants.highRpmAmp);
    highPidController.setReference(highShooterSpeed, ControlType.kVelocity);
    lowShooterSpeed = SmartDashboard.getNumber( "low shooter rpm for Amp", Constants.RobotConstants.lowRpmAmp);
    lowPidController.setReference(lowShooterSpeed, ControlType.kVelocity);
  }

  public void prepareForTrap() {
    highShooterSpeed = SmartDashboard.getNumber( "high shooter rpm for trap", Constants.RobotConstants.highRpmTrap);
    highPidController.setReference(highShooterSpeed, ControlType.kVelocity);
    lowShooterSpeed = SmartDashboard.getNumber( "low shooter rpm for trap", Constants.RobotConstants.lowRpmTrap);
    lowPidController.setReference(lowShooterSpeed, ControlType.kVelocity);
  }

  public void enterIdleMotor() {
    highShooterSpeed = SmartDashboard.getNumber( "high shooter rpm for idle", Constants.RobotConstants.highRpmIdle);
    highPidController.setReference(highShooterSpeed, ControlType.kVelocity);
    lowShooterSpeed = SmartDashboard.getNumber( "low shooter rpm for idle", Constants.RobotConstants.lowRpmIdle);
    lowPidController.setReference(lowShooterSpeed, ControlType.kVelocity);
  }
}
