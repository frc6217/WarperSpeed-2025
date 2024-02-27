// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax leftClimber = new CANSparkMax(41, MotorType.kBrushless);
  CANSparkMax rightClimber = new CANSparkMax(42, MotorType.kBrushless); 
  public Climber() {  
  
    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimber.getEncoder().setPosition(0);
    rightClimber.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void deployLeftClimber(){
  leftClimber.set(-.4); 
  }

  public void deployRightClimber(){
  rightClimber.set(-.4); 
  }

  public void winchLeftClimber(){
    leftClimber.set(.4);
  }

  public void winchRightClimber(){
    rightClimber.set(.4);
  }

  public void stopLeftClimber(){
    leftClimber.set(0);
  }

  public void stopRightClimber(){
    rightClimber.set(0);
  }

  public CANSparkMax getLeftClimber(){
    return leftClimber;
  }

  public CANSparkMax getRightClimber(){
    return rightClimber;
  }  
}
