// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.Shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.Shooter;

public class AutoShootStart extends Command {
  /** Creates a new AutoShootStart. */

 PIDShooter shooter;
 Timer timeout = new Timer();

 // add new debounce
 Debouncer debouncer = new Debouncer(RobotConstants.autoShotDebounceTime);
 boolean debounceIsReady = false;

  public AutoShootStart(PIDShooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.prepareForSpeaker();
    timeout.restart();
    debounceIsReady = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // calculate debounce (using isReady)
    debounceIsReady = debouncer.calculate(shooter.isReady());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // use debounced output here
    
    //make a boolean that is set to timeout advanceIfElpased
    //if true, sys print "shooter timeout hit"
    // OR in the boolean
    boolean hitTimeOut = timeout.advanceIfElapsed(.5);
    if(hitTimeOut){
      System.out.println("shooter timeout hit");
    }
    return debounceIsReady || hitTimeOut;
  }
}
