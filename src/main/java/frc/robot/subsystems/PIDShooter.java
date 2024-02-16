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

public class PIDShooter extends SubsystemBase {
  /** Creates a new PIDShooter. */
  CANSparkMax lowShooter = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax topShooter = new CANSparkMax(6, MotorType.kBrushless);
  SparkPIDController highPidController = topShooter.getPIDController();
  SparkPIDController lowPidController = lowShooter.getPIDController();
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
  }
  public void on(){
    lowPidController.setReference(SmartDashboard.getNumber("lowShooter RPM", 0), ControlType.kVelocity);
    highPidController.setReference(SmartDashboard.getNumber("highShooter RPM", 0), ControlType.kVelocity);
  }
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
}
