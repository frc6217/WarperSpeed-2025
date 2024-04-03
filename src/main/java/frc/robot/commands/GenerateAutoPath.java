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
  ArrayList<JoystickData> driverList = new ArrayList<JoystickData>();
  ArrayList<JoystickData> helperList = new ArrayList<JoystickData>();

  public GenerateAutoPath(CommandXboxController driver, CommandXboxController helper) {
    this.driver = driver;
    this.helper = helper;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    helperList.clear();
    driverList.clear();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    JoystickData j = new JoystickData();
    j.buttons[JoystickData.A_INDEX] = driver.a().getAsBoolean();
    j.buttons[JoystickData.B_INDEX] = driver.b().getAsBoolean();
    j.buttons[JoystickData.X_INDEX] = driver.x().getAsBoolean();
    j.buttons[JoystickData.Y_INDEX] = driver.y().getAsBoolean();
    j.buttons[JoystickData.RB_INDEX] = driver.leftBumper().getAsBoolean();
    j.buttons[JoystickData.LB_INDEX] = driver.rightBumper().getAsBoolean();

    j.joystick[JoystickData.LX_INDEX] = driver.getLeftX();
    j.joystick[JoystickData.LY_INDEX] = driver.getLeftY();
    //j.joystick[JoystickData.RX_INDEX] = driver.a().getAsBoolean();
    //j.joystick[JoystickData.RY_INDEX] = driver.a().getAsBoolean();
    //j.joystick[JoystickData.TRIGGER_INDEX] = driver.a().getAsBoolean();

   // driverList.add(null)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public class JoystickData {
    public static int A_INDEX = 0;
    public static int B_INDEX = 1;
    public static int X_INDEX = 2;
    public static int Y_INDEX = 3;
    public static int RB_INDEX = 4;
    public static int LB_INDEX = 5;
    public static int LX_INDEX = 0;
    public static int LY_INDEX = 1;
    public static int R1_INDEX = 2;
    public static int R2_INDEX = 3;
    public static int TRIGGER_INDEX = 4;
    public double[] joystick = new double[5];
    public boolean[] buttons = new boolean[6];
    public JoystickData(){

    }
  }
}
