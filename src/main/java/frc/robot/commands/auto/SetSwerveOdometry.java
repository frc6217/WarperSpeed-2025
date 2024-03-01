// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class SetSwerveOdometry extends Command {
  /** Creates a new SetSwerveOdometry. */
  SwerveDrivetrain sDrivetrain;

  double xSet;
  double ySet;
  double rotationSet;

  public SetSwerveOdometry(SwerveDrivetrain sDrivetrain, double xSet, double ySet, double rotationSet) {
    this.sDrivetrain = sDrivetrain;
    addRequirements(sDrivetrain);
    this.xSet = xSet;
    this.ySet = ySet;
    this.rotationSet = rotationSet;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //sDrivetrain.sOdometry.resetPosition(null, null, null);
    xSet = Units.feetToMeters(xSet);
    ySet = Units.feetToMeters(ySet);
    
    sDrivetrain.setPigeonAngle(rotationSet);
    sDrivetrain.setOdometry(xSet, ySet);
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
