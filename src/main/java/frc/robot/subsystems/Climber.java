// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private CANSparkMax leftClimber = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax rightClimber = new CANSparkMax(42, MotorType.kBrushless); 

  RelativeEncoder leftEncoder = leftClimber.getEncoder();
  RelativeEncoder rightEncoder = rightClimber.getEncoder();

  //add new limit switch
  DigitalInput leftLimitSwitch = new DigitalInput(RobotConstants.leftLimitSwitchChannel);
  DigitalInput rightLimitSwitch = new DigitalInput(RobotConstants.rightLimitSwitchChannel);
  

  //create a boolean (flag) called overrideLimitSwitch (defaults false)
  private boolean limitSwitchOverride = false;

  public Climber() {  
  
    leftClimber.restoreFactoryDefaults();
    rightClimber.restoreFactoryDefaults();

    leftClimber.getEncoder().setPosition(0);
    rightClimber.getEncoder().setPosition(0);
    
    
    //todo scale encoder if so set softlimit
    
    leftClimber.setIdleMode(IdleMode.kBrake);
    rightClimber.setIdleMode(IdleMode.kBrake);

    leftClimber.setSmartCurrentLimit(60);
    rightClimber.setSmartCurrentLimit(60);

   // leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0)

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //if override is NOT disabled do this:
    if(leftLimitSwitch.get() == true){
      leftClimber.getEncoder().setPosition(0);
      leftClimber.set(0);
    }
    if(rightLimitSwitch.get() == true){
      rightClimber.getEncoder().setPosition(0);
      rightClimber.set(0);
    }
  }

  public double getLeftEncoderValue() {
    // get klet enc value (position)
    return leftEncoder.getPosition();
  }

  // add for right
  public double getRightEncoderValue(){
    return rightEncoder.getPosition();
  }

  public void setLeftClimberSpeed(double speed){
    setLeftClimberSpeedWithLimit(speed);
  }

  public void setRightClimberSpeed(double speed){
    setRightClimberSpeedWithLimit(speed);
  }

  public void deployLeftClimber(){
  setLeftClimberSpeedWithLimit(-.4); 
  }

  public void deployRightClimber(){
  setRightClimberSpeedWithLimit(-.4); 
  }

  public void winchLeftClimber(){
    setLeftClimberSpeedWithLimit(.4);
  }

  public void winchRightClimber(){
    setRightClimberSpeedWithLimit(.4);
  }

  public void stopLeftClimber(){
    setLeftClimberSpeedWithLimit(0);
  }

  public void stopRightClimber(){
    setRightClimberSpeedWithLimit(0);
  } 

  private void setLeftClimberSpeedWithLimit(double speed) {
    if(leftLimitSwitch.get() && speed > 0){
      leftClimber.set(0);
    } else {
      leftClimber.set(speed);
    }
  }
  private void setRightClimberSpeedWithLimit(double speed) {
    if(rightLimitSwitch.get() && speed > 0){
      rightClimber.set(0);
    } else {
      rightClimber.set(speed);
    }
  }

  public void enableLimitSwitchOverride()  {
    //set a boolean (flag)
    limitSwitchOverride = true;

    //leftClimber.setSoftLimit(SoftLimitDirection.kReverse, -1000000) // 
    leftClimber.setSoftLimit(SoftLimitDirection.kReverse, -1000000);
    rightClimber.setSoftLimit(SoftLimitDirection.kReverse, -1000000);
  }

  public void disableLimitSwitchOverride() {
    //clear a boolean (flag)
    limitSwitchOverride = false;
    // set softlimit reverse 0
    leftClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
    rightClimber.setSoftLimit(SoftLimitDirection.kReverse, 0);
  }

  // same for right


  public boolean isLeftLimitPressed() {
    return leftLimitSwitch.get();
    // return true if limit is pressed
  }
  // same for right
  public boolean isRightLimitPressed() {
    return rightLimitSwitch.get();
  }
}
