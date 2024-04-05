// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberSetSpeedCommand extends Command {
  /** Creates a new ClimberSetSpeedCommand. */

  Climber climber;
  double leftSpeed;
  double rightSpeed;

  public ClimberSetSpeedCommand(Climber climber, double leftSpeed, double rightSpeed) {
    this.climber = climber;
    addRequirements(climber);
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
      climber.setLeftClimberSpeed(leftSpeed);
      climber.setRightClimberSpeed(rightSpeed);
      

    //if(0.5 < climber.getLeftClimber())
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftClimberSpeed(0);
    climber.setRightClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
