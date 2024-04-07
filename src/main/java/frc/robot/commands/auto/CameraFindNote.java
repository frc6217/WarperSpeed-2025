// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;

public class CameraFindNote extends Command {
  LimeLightSub ll;
  SwerveDrivetrain drivetrain;
  double direction;

  /** Creates a new CameraFindNote. */
  public CameraFindNote(SwerveDrivetrain drivetrain, LimeLightSub ll, double direction) { // limelight driveTrain
    this.ll = ll;
    this.drivetrain = drivetrain;
    this.direction = direction;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rotate robot
    drivetrain.drive(null, 0.3* direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ll.isValid(); // return true when ll sees object (is valid)
    
  }
}
