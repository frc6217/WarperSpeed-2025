// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberSelector;

public class WinchClimber extends Command {
  /** Creates a new WinchClimber. */
  
  Climber climber;
  ClimberSelector selection;
  
  public WinchClimber(Climber climber, ClimberSelector selection) {
    this.climber = climber;
    this.selection = selection;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    switch (selection) {
      case LEFT:
        climber.winchLeftClimber();
        break;
      case RIGHT:
        climber.winchRightClimber();
        break;
      case BOTH:
        climber.winchRightClimber();
        climber.winchLeftClimber(); 
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopLeftClimber();
    climber.stopRightClimber();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}