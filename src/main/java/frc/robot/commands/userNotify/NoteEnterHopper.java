// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.userNotify;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class NoteEnterHopper extends Command {
  /** Creates a new NoteEnterHopper. */
  Intake intake;
  CommandXboxController controllerToVibrate;
  Timer timer;
  public NoteEnterHopper(Intake intake, CommandXboxController controllerToVibrate) {
    this.controllerToVibrate = controllerToVibrate;
    this.intake = intake;
    //addRequirements(intake);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //intake.intakeOff();
   // intake.ledRainbow();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerToVibrate.getHID().setRumble(RumbleType.kBothRumble,.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerToVibrate.getHID().setRumble(RumbleType.kBothRumble,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.advanceIfElapsed(.3);
  }
}
