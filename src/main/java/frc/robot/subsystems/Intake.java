// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax firstIntake = new CANSparkMax(17, MotorType.kBrushless);
  CANSparkMax secondIntake = new CANSparkMax(14, MotorType.kBrushless);
  
  //LaserCan laser = new LaserCan(55);
  private double laserInches = 0;

  public Intake() {
    SmartDashboard.putNumber("intake1speed", .2);
    SmartDashboard.putNumber("intake2speed", .3);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //laserInches = getLaserInches();
  }
/* 
  private double getLaserInches(){
    return Units.metersToInches((double)(laser.getMeasurement().distance_mm)/1000);
  }
  
  public boolean hasNote(){
    return laserInches < Constants.RobotConstants.laserNoteThresholdInches;
  } */
  public void on1Intake(){
    firstIntake.set(SmartDashboard.getNumber("intake1speed", 0));
  }
  public void off1Intake(){
    firstIntake.set(0);
  }

  public void on2Intake(){
    secondIntake.set(SmartDashboard.getNumber("intake2speed", 0));
  }
  public void off2Intake(){
    secondIntake.set(0);
  }
}
