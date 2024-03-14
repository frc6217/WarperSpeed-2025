// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  CANSparkMax indexer = new CANSparkMax(Constants.RobotConstants.indexerCanId, MotorType.kBrushless);
  DutyCycleEncoder absEncoder = new DutyCycleEncoder(0);
  Shooter shooter;
  double flingDid = 0.145;
  public enum STATE {
    WAIT, FLING, FLINGWAIT, RESET
  };
  STATE state = STATE.WAIT;
  Timer timer = new Timer();

  public Indexer() {
    absEncoder.setPositionOffset(Constants.RobotConstants.indexerStartOffset);
    indexer.setSmartCurrentLimit(RobotConstants.indexerMotorCurrentLimit);
  }

  @Override
  public void periodic() {
    //getAbsolutePosistion or getPosistion????????????????????
    SmartDashboard.putNumber("abs value indexer", absEncoder.getDistance());
    SmartDashboard.putNumber("relative value indexer", indexer.getEncoder().getPosition());
    SmartDashboard.putString("Indexer State", state.toString());
    
    switch (state) {
      case WAIT:
        break;
      case FLING:
      {
        if (absEncoder.getDistance() < flingDid){
          indexer.set(-.2);
        } else{
          indexer.set(0);
          state = STATE.FLINGWAIT;
          timer.reset();
          timer.start();
        }
          
        break;
      }
      case FLINGWAIT:
      {
        if(timer.hasElapsed(.1)){
          state = STATE.RESET;
        }
        break;
      }
      case RESET:
      {
        if (absEncoder.getDistance() > .05){
          indexer.set(.2);
        } else{
          indexer.set(0);
          state = STATE.WAIT;
        }

        break;
      }
    
      default:
        break;
    }
    // This method will be called once per scheduler run
  }
  public void shoot(){
    if (state == STATE.WAIT){
      state = STATE.FLING;
    }
  }

}
