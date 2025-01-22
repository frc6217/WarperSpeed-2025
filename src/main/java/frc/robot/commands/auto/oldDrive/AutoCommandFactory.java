// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.oldDrive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;


public class AutoCommandFactory {
    SwerveDrivetrain sDrivetrain;
    LimeLightSub limeLightSub;
    RobotContainer robotContainer;

    //Potenial Maybe Use
    // public SendableChooser<Command> autoChooserStep1 = new SendableChooser<Command>();
    // public SendableChooser<Command> autoChooserStep2 = new SendableChooser<Command>();
    public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    SequentialCommandGroup testCommand;

    public AutoCommandFactory(RobotContainer robotContainer,SwerveDrivetrain sDrivetrain, LimeLightSub limeLightSub){
        this.sDrivetrain = sDrivetrain;
        this.limeLightSub = limeLightSub;
        this.robotContainer = robotContainer;

        testCommand = new SequentialCommandGroup();

         SmartDashboard.putData(autoChooser);

    }
    public Command getAutoCommand(){
      return autoChooser.getSelected();
      }

   
    }