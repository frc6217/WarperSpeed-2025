// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GenerateAutoPath extends Command {
  /** Creates a new GenerateAutoPath. */
  CommandXboxController driver, helper;
  ArrayList driverList = new ArrayList<>();
  public GenerateAutoPath(CommandXboxController driver, CommandXboxController helper) {
    this.driver = driver;
    this.helper = helper;
    // Use addRequirements() here to declare subsystem dependencies.
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

  public class JoystickData {
    public double[] joystick = new double[5];
    public boolean[] buttons = new boolean[6];
    //public JoystickData()
  }
}
