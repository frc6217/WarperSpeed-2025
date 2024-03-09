// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AllianceSelector extends SubsystemBase {
  /** Creates a new AllianceSelector. */
  private Alliance alliance;
  public AllianceSelector() {
    setAlliance();
  }

  public void setAlliance(){
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            alliance = Alliance.Red;
        }
        if (ally.get() == Alliance.Blue) {
            alliance = Alliance.Blue;
        }
    }
    else {
        System.out.println("No Alliance Given");
        alliance = Alliance.Red;
    }
  }

  @Override
  public void periodic() {
   
    // This method will be called once per scheduler run
  }

  public AprilTag getMiddleSpeakerAprilTag(){
    if(alliance == Alliance.Red){
      return new AprilTag(4, null);
    } else{
      return new AprilTag(7, null);
    }
  }

  public AprilTag getSideSpeakerAprilTag(){
    if(alliance == Alliance.Red){
      return new AprilTag(3, null);
    } else{
      return new AprilTag(8, null);
    }
  }

  public AprilTag getAmpAprilTag(){
    if(alliance == Alliance.Red){
      return new AprilTag(5, null);
    } else{
      return new AprilTag(6, null);
    }
  }

  public AprilTag getNearSourceAprilTag(){
    if(alliance == Alliance.Red){
      return new AprilTag(10, null);
    } else{
      return new AprilTag(1, null);
    }
  }

  public AprilTag getFarSourceAprilTag(){
    if(alliance == Alliance.Red){
      return new AprilTag(9, null);
    } else{
      return new AprilTag(2, null);
    }
  }

  public Alliance getAllianceColor(){
    return alliance;
  }
}
