// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class AbsoluteDiseredDriveNoPID extends Command {
  /** Creates a new DriveXfeetYfeet. */
  SwerveDrivetrain sDrivetrain;


  double xSetpoint;
  double ySetpoint;

  double rotationSetpoint;

  boolean rotateBoolean;

  double outputTranslation = 0;
  double outputStrafe = 0;
  double outputRotation = 0;
  double n = 0;
  double currentAngle;

  public AbsoluteDiseredDriveNoPID(double xSetpoint, double ySetpoint, double rotationDegreeSetpoint, SwerveDrivetrain sDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
    this.rotationSetpoint = rotationDegreeSetpoint;
    rotateBoolean = true;
  }

  public AbsoluteDiseredDriveNoPID(double xSetpoint, double ySetpoint, SwerveDrivetrain sDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
    rotateBoolean = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(rotateBoolean){
      rotationSetpoint = rotationSetpoint;
    }else {
      rotationSetpoint = sDrivetrain.getWrapedAngle();
    
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    SmartDashboard.putNumber("Current Angle Wrapping", currentAngle);
    currentAngle = sDrivetrain.getWrapedAngle();

    if(Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()) - xSetpoint) < .4){
      outputTranslation = 0;
    }else if((Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()) - xSetpoint) > .2){
      outputTranslation = -.6;
    }else{
      outputTranslation = .6;
    }

    if(Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()) - ySetpoint) < .4){
      outputStrafe = 0;
    }else if((Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()) - ySetpoint) > .2){
      outputStrafe = -.6;
    }else{
      outputStrafe = .6;
    }
    
  
    if(Math.abs(currentAngle - rotationSetpoint) < 8){
      outputRotation = 0;
    }else if((currentAngle < rotationSetpoint)){
        if((rotationSetpoint - currentAngle) < 180){
        outputRotation = .07;
        }else{
        outputRotation = -.07;
      }
    }else{
      if((currentAngle - rotationSetpoint) < 180){
        outputRotation = -.06;
        }else{
        outputRotation = .06;
      }
    }
  // scale up with maxVelo
    sDrivetrain.drive(new Translation2d(outputTranslation, outputStrafe).times(Constants.RobotConstants.driveMaxVelo), outputRotation*Constants.RobotConstants.rotationMaxAngleVelo);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sDrivetrain.drive(new Translation2d(0,0), 0);
   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean val1 = Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()) - xSetpoint) < .2; 
    boolean val2 = Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()) - ySetpoint) < .2;
    boolean val3 = Math.abs(currentAngle - rotationSetpoint) < 5;
    return val1 && val2 && val3 ;
  }
}
