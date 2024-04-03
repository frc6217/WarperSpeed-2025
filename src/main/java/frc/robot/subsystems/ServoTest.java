// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoTest extends SubsystemBase {
  /** Creates a new ServoTest. */
  Servo s = new Servo(0);
  Timer t = new Timer();
  boolean isZero = false;
  public ServoTest() {
    t.start();
    t.restart();
  }

  @Override
  public void periodic() {

    // move servo
    // wait X seconds
    // move servo some where else
    // wait X seconds
    //repeat

    if(t.hasElapsed(2)){
      t.restart();
      if (isZero){
        s.setAngle(179);
        isZero = false;
      } else{
        s.setAngle(1);
        isZero = true;
      }
    }

    // This method will be called once per scheduler run
  }
}
