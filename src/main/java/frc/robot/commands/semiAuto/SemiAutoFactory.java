// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ThirdIntakeCommand;
import frc.robot.commands.auto.Intake.AutoIntakeEnd;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class SemiAutoFactory {
    RobotContainer robot;
    public SemiAutoFactory(RobotContainer robot){
        this.robot = robot;
    }



    public Command autoPickupNote(){
        ParallelDeadlineGroup group = new ParallelDeadlineGroup(new CameraDrive(robot.swerveDrivetrain, robot.noteFinderLimeLight, Constants.SemiAutoConstants.note, robot.intake),
             new IntakeCommand(robot.intake,.8), new ThirdIntakeCommand(robot.thirdIntakeWheels, RobotConstants.thridIntakeSpeed));
        return group;
    }

     public ParallelDeadlineGroup speakerGoTo(){
        ParallelDeadlineGroup group = new ParallelDeadlineGroup(new CameraDrive(robot.swerveDrivetrain, robot.shooterLimeLight, Constants.SemiAutoConstants.speaker, robot.intake));
        return group;
    }

    public Command autoPickupUntilSignal(){
        ParallelRaceGroup group = new ParallelRaceGroup(new CameraDrive(robot.swerveDrivetrain, robot.noteFinderLimeLight, Constants.SemiAutoConstants.note, robot.intake), new IntakeCommand(robot.intake,.8));
        group.until(robot.intake::haveNote);
        return group;
    }
}
