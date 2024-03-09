// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  CANSparkMax lowShooter = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax topShooter = new CANSparkMax(6, MotorType.kBrushless);
  /** Creates a new Shooter. */
  public Shooter() {
    SmartDashboard.putNumber("lowShooterspeed", -.96);
    SmartDashboard.putNumber("highShooterspeed", -.86);
    topShooter.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void on() {
    lowShooter.set(SmartDashboard.getNumber("lowShooterspeed", 0));
    topShooter.set(SmartDashboard.getNumber("highShooterspeed", 0));
    // lowShooter.set(-0.7);
    // topShooter.set(-0.7);
  }

  public void off() {
    lowShooter.set(0);
    topShooter.set(0);
  }

  public void speedShoot(double topSpeed, double lowSpeed){
    lowShooter.set(lowSpeed);
    topShooter.set(topSpeed);
  }
}
