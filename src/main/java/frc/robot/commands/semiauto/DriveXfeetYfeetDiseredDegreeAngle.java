// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiauto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class DriveXfeetYfeetDiseredDegreeAngle extends Command {
  /** Creates a new DriveXfeetYfeet. */
  SwerveDrivetrain sDrivetrain;

  Double initialX;
  Double initialY;

  Double xSetpoint;
  Double ySetpoint;

  Double rotationSetpoint;

  PIDController xPidController = new PIDController(.2, 0, 0);
  PIDController yPidController = new PIDController(.2, 0, 0);
  PIDController rotatoinPidController = new PIDController(.2, 0, 0);

  public DriveXfeetYfeetDiseredDegreeAngle(Double xSetpoint, Double ySetpoint, Double rotationDegreeSetpoint, SwerveDrivetrain sDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
    this.rotationSetpoint = rotationDegreeSetpoint;
  }

  public DriveXfeetYfeetDiseredDegreeAngle(Double xSetpoint, Double ySetpoint, SwerveDrivetrain sDrivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
    this.rotationSetpoint = sDrivetrain.getAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double initialX = Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX());
    double initialY = Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY());
   
    xSetpoint = xSetpoint + initialX;
    ySetpoint = ySetpoint + initialY;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xPidController.setSetpoint(xSetpoint);
    yPidController.setSetpoint(ySetpoint);

    rotatoinPidController.setSetpoint(rotationSetpoint);

    double outputTranslation = xPidController.calculate(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()));
    double outputStrafe = yPidController.calculate(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()));
    double outputRotation = rotatoinPidController.calculate(sDrivetrain.getAngle());

    sDrivetrain.drive(new Translation2d(outputTranslation, outputStrafe), outputRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sDrivetrain.drive(new Translation2d(0,0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (
    Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getX()) - xSetpoint) < .1 && 
    Math.abs(Units.metersToFeet(sDrivetrain.sOdometry.getPoseMeters().getY()) - ySetpoint) < .1 &&
    Math.abs(sDrivetrain.getAngle() - rotationSetpoint) < 1
    );
  }
}
