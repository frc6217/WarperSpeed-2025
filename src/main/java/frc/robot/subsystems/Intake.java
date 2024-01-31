// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax firstIntake = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax secondIntake = new CANSparkMax(0, MotorType.kBrushless);
  
  LaserCan laser = new LaserCan(55);
  private double laserInches = 0;

  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    laserInches = getLaserInches();
  }

  private double getLaserInches(){
    return Units.metersToInches((double)(laser.getMeasurement().distance_mm)/1000);
  }
  
  public boolean hasNote(){
    return laserInches < Constants.RobotConstants.laserNoteThresholdInches;
  }
}
