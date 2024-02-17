// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiauto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;

public class NoteAutoPickup extends Command {
  /** Creates a new noteAutoPickup. */
  SwerveDrivetrain sDrivetrain;
  Intake intake;
  LimeLightSub limelight;

  PIDController translationPidController = new PIDController(0.1,0,0);
  PIDController strafePidController = new PIDController(0.1,0,0);
  PIDController rotationPidController = new PIDController(0.1,0,0)

  public NoteAutoPickup(SwerveDrivetrain sDrivetrain, Intake intake, LimeLightSub limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sDrivetrain = sDrivetrain;
    this.intake = intake;
    this.limelight = limelight;
    addRequirements(sDrivetrain, intake, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
