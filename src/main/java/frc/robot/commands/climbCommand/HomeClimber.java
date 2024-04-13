// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommand;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class HomeClimber extends Command {
  
  // create climber variable
  Climber climber;

  public HomeClimber(/*pass in climber*/ Climber climber) {
    this.climber = climber;
    addRequirements(climber);
    //addRquirements climber
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // enable the overrides
    climber.enableLimitSwitchOverride();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // set speed if limit switch is NOT pressed
    if(climber.isLeftLimitPressed() == false) {
      climber.winchLeftClimber();
    }
    if(climber.isRightLimitPressed() == false) {
      climber.winchRightClimber();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shut off both climbers
    climber.disableLimitSwitchOverride();
    climber.setLeftClimberSpeed(0);
    climber.setRightClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.isLeftLimitPressed() && climber.isRightLimitPressed(); // return true if left and right limits are pressed
  }
}
