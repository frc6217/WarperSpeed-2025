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
import frc.robot.Constants.SemiAutoConstants;

public class PIDShooter extends SubsystemBase {
  /** Creates a new PIDShooter. */
  CANSparkMax lowShooter = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax topShooter = new CANSparkMax(6, MotorType.kBrushless);
  SparkPIDController highPidController = topShooter.getPIDController();
  SparkPIDController lowPidController = lowShooter.getPIDController();


  ShooterSetPoints setPointsToUse = new ShooterSetPoints(0, 0);

  public PIDShooter() {
    SmartDashboard.putNumber("lowShooter RPM", -4980);
    SmartDashboard.putNumber("highShooter RPM", -4570);
    SmartDashboard.putNumber("low P", .00030);
    SmartDashboard.putNumber("low I", 0.000001);
    SmartDashboard.putNumber("low D", 0);
    SmartDashboard.putNumber("high P", .0003);
    SmartDashboard.putNumber("high I", 0.000001);
    SmartDashboard.putNumber("high D", 0);
    topShooter.setInverted(true);
    topShooter.setSmartCurrentLimit(RobotConstants.shooterMotorCurrentLimit);
    lowShooter.setSmartCurrentLimit(RobotConstants.shooterMotorCurrentLimit);
    topShooter.enableVoltageCompensation(RobotConstants.shooterVoltageCompensation);
    lowShooter.enableVoltageCompensation(RobotConstants.shooterVoltageCompensation);
    
    lowPidController.setP(SmartDashboard.getNumber("low P", 0));
    lowPidController.setI(SmartDashboard.getNumber("low I", 0));
    lowPidController.setD(SmartDashboard.getNumber("low D", 0));
    highPidController.setP(SmartDashboard.getNumber("high P", 0));
    highPidController.setI(SmartDashboard.getNumber("high I", 0.00000));
    highPidController.setD(SmartDashboard.getNumber("high D", 0));

  }
  /*
  public void on(){
    lowPidController.setReference(SmartDashboard.getNumber("lowShooter RPM", 0), ControlType.kVelocity);
    highPidController.setReference(SmartDashboard.getNumber("highShooter RPM", 0), ControlType.kVelocity);
  }
*/
  @Override
  public void periodic() {
   /*
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
    highPidController.setI(SmartDashboard.getNumber("high I", 0.000001));
   }
   if (SmartDashboard.getNumber("high D", 0) != highPidController.getD()){
    highPidController.setD(SmartDashboard.getNumber("high D", 0));
   } */
   SmartDashboard.putNumber("High Shooter Current RPM:", topShooter.getEncoder().getVelocity());
   SmartDashboard.putNumber("Low Shooter Current RPM:", lowShooter.getEncoder().getVelocity());

  // highPidController.setReference(-4920, ControlType.kVelocity);
  //lowPidController.setReference(-4580, ControlType.kVelocity);
  highPidController.setReference(setPointsToUse.topShooterSetPoint, ControlType.kVelocity);
  lowPidController.setReference(setPointsToUse.bottomShooterSetPoint, ControlType.kVelocity);

  }


  public boolean isReady() {
    boolean isTopSpeedReady =  (Math.abs(topShooter.getEncoder().getVelocity() - setPointsToUse.topShooterSetPoint)/setPointsToUse.topShooterSetPoint) < .05;
    boolean isLowSpeedReady =  (Math.abs(lowShooter.getEncoder().getVelocity() - setPointsToUse.bottomShooterSetPoint)/setPointsToUse.bottomShooterSetPoint) < .05;
    return isLowSpeedReady && isTopSpeedReady;
  }

  public void prepareForSpeaker() {
    setPointsToUse = SemiAutoConstants.speakerSetPoints;
    lowPidController.setIAccum(0);
    highPidController.setIAccum(0);
  }

  public void prepareForAmp() {
    setPointsToUse = SemiAutoConstants.ampSetPoints;
    lowPidController.setIAccum(0);
    highPidController.setIAccum(0);
  }

  public void prepareForTrap() {
    setPointsToUse = SemiAutoConstants.trapSetPoints;
    lowPidController.setIAccum(0);
    highPidController.setIAccum(0);
  }

  public void enterIdleMotor() {
    setPointsToUse = SemiAutoConstants.idleSetPoints;
    lowPidController.setIAccum(0);
    highPidController.setIAccum(0);
  }

  public void off() {
    setPointsToUse = SemiAutoConstants.offSetPoints;
    lowShooter.set(0);
    topShooter.set(0);
    lowPidController.setIAccum(0);
    highPidController.setIAccum(0);
  }


  public static class ShooterSetPoints {
    public double topShooterSetPoint;
    public double bottomShooterSetPoint;

    public ShooterSetPoints(double topShooterSetPoint, double bottomShooterSetPoint) {
      this.topShooterSetPoint = topShooterSetPoint;
      this.bottomShooterSetPoint = bottomShooterSetPoint;
    }
  }


}


