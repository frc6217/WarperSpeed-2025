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

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  CANSparkMax firstIntake = new CANSparkMax(17, MotorType.kBrushless);
  CANSparkMax secondIntake = new CANSparkMax(14, MotorType.kBrushless);

  Counter noteDetector = new Counter(Counter.Mode.kSemiperiod);

  CANdle candle = new CANdle(51);
  int numLeds = 68;

  AllianceSelector allianceSelector;
  
  public Intake(SwerveDrivetrain drivetrain) {
    noteDetector.setUpSource(1);
    noteDetector.setSemiPeriodMode(true);

    SmartDashboard.putNumber("intake1speed", .2);
    SmartDashboard.putNumber("intake2speed", .3);

    this.allianceSelector = drivetrain.allianceSelector;

    new Trigger(this::haveNote).onFalse(Commands.runOnce(this::intakeOn, this));
    new Trigger(this::haveNote).onTrue(Commands.runOnce(this::intakeOff, this));

    new Trigger(this::haveNote).onTrue(Commands.runOnce(this::ledRainbow, this));
    new Trigger(this::haveNote).onFalse(Commands.runOnce(this::ledNoNote, this));

    this.ledrgbFade();

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("dectector3: ", haveNote());
    // This method will be called once per scheduler run

  }

  public void on1Intake(){
    firstIntake.set(SmartDashboard.getNumber("intake1speed", 0));
  }

  public void reverse1Intake(){
    firstIntake.set(-SmartDashboard.getNumber("intake1speed", 0));
  }

  public void off1Intake(){
    firstIntake.set(0);
  }

  public void on2Intake(){
    secondIntake.set(SmartDashboard.getNumber("intake2speed", 0));
  }

   public void reverse2Intake(){
    secondIntake.set(-SmartDashboard.getNumber("intake2speed", 0));
  }

  public void off2Intake(){
    secondIntake.set(0);
  }

  public void intakeOn(){
    firstIntake.set(.65);
    secondIntake.set(-.65);
  }
  
  public void intakeReverse(){
    firstIntake.set(-.5);
    secondIntake.set(-.5);
  }

  public void intakeOff(){
    firstIntake.set(0);
    secondIntake.set(0);
  }
  public void setSpeed(double speed) {
    firstIntake.set(speed);
    secondIntake.set(-speed);
  }

  public boolean haveNote() {
    // return true if note is detected
    return noteDetector.getPeriod() < Constants.RobotConstants.noteDetectorThreshold;
  }

  public void ledRainbow(){
    RainbowAnimation rainbow = new RainbowAnimation();
    candle.animate(rainbow);
  }
  public void ledBlue(){
    //TwinkleAnimation animate = new TwinkleAnimation(0, 0, 255);
    SingleFadeAnimation animate = new SingleFadeAnimation(0, 0, 255, 0, .5, numLeds);
    candle.animate(animate);
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
    candle.animate(animate);
  }


  public void ledrgbFade() {
    RgbFadeAnimation animate = new RgbFadeAnimation(1, .5, numLeds);
    candle.animate(animate);
  }


  public double noteSensorDistance() {
    // return distance in inches from sensor
    double timeMeasurement = noteDetector.getPeriod() * 1000000;
    double mm = 0.75 * ((timeMeasurement) - 1000);
    return mm/25.4;
    }
  }