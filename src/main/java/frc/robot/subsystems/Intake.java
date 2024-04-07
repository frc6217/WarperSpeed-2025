// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.sensors.HopperBeamBreak;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax firstIntake = new CANSparkMax(Constants.RobotConstants.firstIntakeCanId, MotorType.kBrushless);
  CANSparkMax secondIntake = new CANSparkMax(Constants.RobotConstants.secondIntakeCanId, MotorType.kBrushless);
  CANSparkMax fourthInTake =  new CANSparkMax(Constants.RobotConstants.fourthIntakeCanId, MotorType.kBrushless);
    //pulse distance sesnor
  //Counter noteDetector = new Counter(Counter.Mode.kSemiperiod);



  //CANdle candle = new CANdle(Constants.RobotConstants.candleCanId, "CTRSwerve");
  int numLeds = 168;

  AllianceSelector allianceSelector;
  HopperBeamBreak beamNoteDetector;

  boolean hasNote = true;
  
  public Intake(SwerveDrivetrain drivetrain, HopperBeamBreak beamNoteDetector) {
    // noteDetector.setUpSource(1);
    // noteDetector.setSemiPeriodMode(true);

    // SmartDashboard.putNumber("intake1speed", .2);
    // SmartDashboard.putNumber("intake2speed", .3);

    this.allianceSelector = drivetrain.allianceSelector;
    this.beamNoteDetector = beamNoteDetector;

    

    this.ledrgbFade();

    firstIntake.setSmartCurrentLimit(RobotConstants.intakeMotorCurrentLimit);
    secondIntake.setSmartCurrentLimit(RobotConstants.intakeMotorCurrentLimit-10);
    fourthInTake.setSmartCurrentLimit(RobotConstants.intakeMotorCurrentLimit);

    firstIntake.setIdleMode(IdleMode.kCoast);
    secondIntake.setIdleMode(IdleMode.kCoast);
    fourthInTake.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("dectector3: ", haveNote());
    // SmartDashboard.putNumber("whatever", noteDetector.getPeriod());
    // // This method will be called once per scheduler run
    hasNote = beamNoteDetector.get();

    if(hasNote){
      ledRainbow();
    }else{
      ledNoNote();
    }
  }

  // public void on1Intake(){
  //   firstIntake.set(SmartDashboard.getNumber("intake1speed", 0));
  // }

  // public void reverse1Intake(){
  //   firstIntake.set(-SmartDashboard.getNumber("intake1speed", 0));
  // }

  // public void off1Intake(){
  //   firstIntake.set(0);
  // }

  // public void on2Intake(){
  //   secondIntake.set(SmartDashboard.getNumber("intake2speed", 0));
  // }

  //  public void reverse2Intake(){
  //   secondIntake.set(-SmartDashboard.getNumber("intake2speed", 0));
  // }

  // public void off2Intake(){
  //   secondIntake.set(0);
  // }

  // public void intakeOn(){
  //   firstIntake.set(.65);
  //   secondIntake.set(-.65);

  // }
  
  // public void intakeReverse(){
  //   firstIntake.set(-.5);
  //   secondIntake.set(.5);
  // }

  public void intakeOff(){
    firstIntake.set(0);
    secondIntake.set(0);
    fourthInTake.set(0);
  }
  public void setSpeed(double speed) {
    firstIntake.set(speed);
    secondIntake.set(-speed);
    fourthInTake.set(speed);
  }

  public boolean haveNote() {
    // return true if note is detected
    //return noteDetector.getPeriod() < Constants.RobotConstants.noteDetectorThreshold;
    return hasNote;
  }

  public void ledRainbow(){
    RainbowAnimation rainbow = new RainbowAnimation();
   // candle.animate(rainbow);
  }
  public void ledBlue(){
    //TwinkleAnimation animate = new TwinkleAnimation(0, 0, 255);
    SingleFadeAnimation animate = new SingleFadeAnimation(0, 0, 255, 0, .5, numLeds);
    //candle.animate(animate);
  }

  public void ledNoNote() {
    if (allianceSelector.getAllianceColor() == Alliance.Red) {
      ledRed();
    } else {
      ledBlue();
    }
  }

    public void ledRed(){
    //TwinkleAnimation animate = new TwinkleAnimation(0, 0, 255);
    SingleFadeAnimation animate = new SingleFadeAnimation(255, 0, 0, 0, .5, numLeds);
   // candle.animate(animate);
  }


  public void ledrgbFade() {
    RgbFadeAnimation animate = new RgbFadeAnimation(1, .5, numLeds);
    //candle.animate(animate);
  }

  /* 

  public double noteSensorDistance() {
    // return distance in inches from sensor
    double timeMeasurement = noteDetector.getPeriod() * 1000000;
    double mm = 0.75 * ((timeMeasurement) - 1000);
    return mm/25.4;
    }
    */
  }