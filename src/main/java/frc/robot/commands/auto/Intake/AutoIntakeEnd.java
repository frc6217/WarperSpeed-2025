// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ThirdIntakeWheels;

public class AutoIntakeEnd extends Command {
  /** Creates a new AutoIntakeEnd. */
  Intake intake;
  ThirdIntakeWheels thirdIntakeWheels;

  public AutoIntakeEnd(Intake intake, ThirdIntakeWheels thirdIntakeWheels) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.thirdIntakeWheels = thirdIntakeWheels;
    addRequirements(intake, thirdIntakeWheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setSpeed(0);
    thirdIntakeWheels.setSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
