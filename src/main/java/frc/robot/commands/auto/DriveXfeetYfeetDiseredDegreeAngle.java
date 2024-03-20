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

public class DriveXfeetYfeetDiseredDegreeAngle extends Command {
  /** Creates a new DriveXfeetYfeet. */
  SwerveDrivetrain sDrivetrain;

  double initialX;
  double initialY;

  double xSetpoint;
  double ySetpoint;

  double rotationSetpoint;

  PIDController xPidController = new PIDController(0, 0, 0);
  PIDController yPidController = new PIDController(0, 0, 0);
  PIDController rotationPidController = new PIDController(0, 0, 0);

  boolean rotateBoolean;

  public DriveXfeetYfeetDiseredDegreeAngle(double xSetpoint, double ySetpoint, double rotationDegreeSetpoint, SwerveDrivetrain sDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
    this.rotationSetpoint = rotationDegreeSetpoint;
    rotateBoolean = true;
     SmartDashboard.putNumber("Auto xPID P", 0.1);
    SmartDashboard.putNumber("Auto yPID P", 0.1);
    SmartDashboard.putNumber("Auto xPID I", 0.02);
    SmartDashboard.putNumber("Auto yPID I", 0.02);
    SmartDashboard.putNumber("Auto xPID D", 0.02);
    SmartDashboard.putNumber("Auto yPID D", 0.02);
    SmartDashboard.putNumber("Auto rotationPID I", 0);
    SmartDashboard.putNumber("Auto rotationPID D", 0);
    SmartDashboard.putNumber("Auto rotationPID P", 0.0036);
  }

  public DriveXfeetYfeetDiseredDegreeAngle(double xSetpoint, double ySetpoint, SwerveDrivetrain sDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
    rotateBoolean = false;
    SmartDashboard.putNumber("Auto xPID P", 0.5);
    SmartDashboard.putNumber("Auto yPID P", 0.5);
    SmartDashboard.putNumber("Auto xPID I", 0.02);
    SmartDashboard.putNumber("Auto yPID I", 0.02);
    SmartDashboard.putNumber("Auto xPID D", 0.02);
    SmartDashboard.putNumber("Auto yPID D", 0.02);
    SmartDashboard.putNumber("Auto rotationPID I", 0);
    SmartDashboard.putNumber("Auto rotationPID D", 0);
    SmartDashboard.putNumber("Auto rotationPID P", 0.0036);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(rotateBoolean){
      rotationSetpoint = rotationSetpoint;
    }else {
      rotationSetpoint = sDrivetrain.getAngle();
    }
    
    xPidController.setP(SmartDashboard.getNumber("Auto xPID P", 0));
    yPidController.setP(SmartDashboard.getNumber("Auto yPID P", 0));
    xPidController.setI(SmartDashboard.getNumber("Auto xPID I", 0));
    yPidController.setI(SmartDashboard.getNumber("Auto yPID I", 0));
    xPidController.setD(SmartDashboard.getNumber("Auto xPID D", 0));
    yPidController.setD(SmartDashboard.getNumber("Auto yPID D", 0));
    rotationPidController.setP(SmartDashboard.getNumber("Auto rotationPID P", 0));
    rotationPidController.setI(SmartDashboard.getNumber("Auto rotationPID I", 0));
    rotationPidController.setD(SmartDashboard.getNumber("Auto rotationPID D", 0));
 
    xPidController.reset();
    yPidController.reset();
    rotationPidController.reset();

    xPidController.setSetpoint(xSetpoint);
    yPidController.setSetpoint(ySetpoint);
    rotationPidController.setSetpoint(rotationSetpoint);

    rotationPidController.enableContinuousInput(0, 360);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    double outputTranslation = xPidController.calculate(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()));
    double outputStrafe = yPidController.calculate(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()));
    double outputRotation = rotationPidController.calculate(sDrivetrain.getAngle());


    outputRotation = MathUtil.clamp(outputRotation, -1, 1);
    outputStrafe = MathUtil.clamp(outputStrafe, -1, 1);
    outputTranslation = MathUtil.clamp(outputTranslation, -1, 1);
    
    SmartDashboard.putNumber("Auto Translation Output", outputTranslation);
    SmartDashboard.putNumber("Auto Strafe Output", outputStrafe);
    SmartDashboard.putNumber("Auto Rotation Output", outputRotation);

    SmartDashboard.putNumber("Auto Translation Setpoint", xSetpoint);
    SmartDashboard.putNumber("Auto Strafe Setpoint", ySetpoint);
    SmartDashboard.putNumber("Auto Rotation Setpoint", rotationSetpoint);

    SmartDashboard.putNumber("Auto Translation Current", Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()));
    SmartDashboard.putNumber("Auto Strafe Current", Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()));
    SmartDashboard.putNumber("Auto Rotation Current", sDrivetrain.getAngle());

    SmartDashboard.putNumber("Auto Translation P", xPidController.getP());

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
    boolean val1 = Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()) - xSetpoint) < .4; 
    boolean val2 = Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()) - ySetpoint) < .4;
    boolean val3 = Math.abs(sDrivetrain.getAngle() - rotationSetpoint) < 4;
    return val1 && val2 && val3 ;
  }
}
