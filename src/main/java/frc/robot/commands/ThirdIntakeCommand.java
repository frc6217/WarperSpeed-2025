// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ThirdIntakeWheels;

public class ThirdIntakeCommand extends Command {
  /** Creates a new ThirdIntakeWheels. */
  ThirdIntakeWheels thirdIntakeWheels;
  Double speed;

  public ThirdIntakeCommand(ThirdIntakeWheels thirdIntakeWheels, Double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.thirdIntakeWheels = thirdIntakeWheels;
    this.speed = speed;
    addRequirements(thirdIntakeWheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    thirdIntakeWheels.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    thirdIntakeWheels.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
