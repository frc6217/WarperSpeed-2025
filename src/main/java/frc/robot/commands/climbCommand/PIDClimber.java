// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Climber;

public class PIDClimber extends Command {
  /** Creates a new PIDClimber. */

  // create 2 new pid controllers
  PIDController leftPidController;
  PIDController righPidController;

  // add subsytem
  Climber climber;
  SetPoints setPoints;

  public PIDClimber(Climber climber, SetPoints setPoints) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    //add subsystem
    this.climber = climber;
    this.setPoints = setPoints;
    // make new pid controller
    leftPidController = new PIDController(0, 0, 0);
    righPidController =  new PIDController(0, 0, 0);
    // pass in two set points

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // pid tolarance and setSetPoint
    leftPidController.setTolerance(10);
    leftPidController.setSetpoint(setPoints.leftSetPoint);

    righPidController.setTolerance(10);
    righPidController.setSetpoint(setPoints.rightSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // calculate pids and set output of pid to climber subsystem (speed)
    double leftClimberAmount = leftPidController.calculate(climber.getLeftEncoderValue());
    leftClimberAmount = MathUtil.clamp(leftClimberAmount, -.5, .5);

    double rightClimberAmount = righPidController.calculate(climber.getRightEncoderValue());
    rightClimberAmount = MathUtil.clamp(rightClimberAmount, -.5, .5);

    climber.setLeftClimberSpeed(leftClimberAmount);                                                                                                            
    climber.setRightClimberSpeed(rightClimberAmount);
    //clamp!
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // speed to 0!
    climber.setLeftClimberSpeed(0);
    climber.setRightClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return true if both climbers are at right height
    return leftPidController.atSetpoint() && righPidController.atSetpoint();
  }

  public class SetPoints {
    public int leftSetPoint;
    public int rightSetPoint;
    public SetPoints(int left, int right) {
      this.leftSetPoint = left;
      this.rightSetPoint = right;
    }
  }
}
