// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrivetrain;

public class FindKS extends Command {
  /** Creates a new FindKS. */
  CANSparkMax motor;
  double initialCount;
  double voltage;

  public FindKS(CANSparkMax motor, SwerveDrivetrain dt) {
    this.motor = motor;
    addRequirements(dt);
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialCount = motor.getEncoder().getPosition();
    voltage = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    voltage = voltage + 0.01;
    motor.set(voltage);
    System.out.println(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return initialCount != motor.getEncoder().getPosition();
  }
}
