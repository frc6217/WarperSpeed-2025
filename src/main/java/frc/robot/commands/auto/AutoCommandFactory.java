// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.semiauto.AbsoluteDiseredDriveNoPID;
import frc.robot.commands.semiauto.RelativeDiseredDriveNoPID;
import frc.robot.commands.shootCommands.SpeakerShootCommand;
import frc.robot.commands.shootCommands.StopShooterCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoCommandFactory {
    SwerveDrivetrain sDrivetrain;
    Indexer indexer;
    Intake intake;
    Shooter shooter;
    LimeLightSub limeLightSub;
    

    //Potenial Maybe Use
    // public SendableChooser<Command> autoChooserStep1 = new SendableChooser<Command>();
    // public SendableChooser<Command> autoChooserStep2 = new SendableChooser<Command>();
    public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    SequentialCommandGroup testCommand;

    public AutoCommandFactory(SwerveDrivetrain sDrivetrain, Indexer indexer, Intake intake, Shooter shooter, LimeLightSub limeLightSub){
        this.sDrivetrain = sDrivetrain;
        this.indexer = indexer;
        this.shooter = shooter;
        this.limeLightSub = limeLightSub;
        this.intake = intake;

        testCommand = new SequentialCommandGroup();

        autoChooser.setDefaultOption("No Shot, no move", AlwaysDo().andThen(IntakeOff()));

        autoChooser.addOption("Middle, One Shot, No Move ", AlwaysDo().andThen(doAutoFirstShot()));
        autoChooser.addOption("Middle, One Shot, Simple Backup", AlwaysDo().andThen(doAutoFirstShot()).andThen(doSimpleBackUp()));
        autoChooser.addOption("Middle, Two Shot, Score Near Middle Note", AlwaysDo().andThen(doAutoFirstShot()).andThen(GoToNearMiddleNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()));

         SmartDashboard.putData(autoChooser);
    }
    public /*SequentialCommandGroup*/ Command getAutoCommand(){
      return autoChooser.getSelected();
      }
    
    public SequentialCommandGroup AlwaysDo(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(Commands.runOnce(sDrivetrain::enableBrakes, sDrivetrain));
        pCommandGroup.addCommands(Commands.run(intake::intakeOn, intake));
        return pCommandGroup;
      }

    public SequentialCommandGroup IntakeOff(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(Commands.run(intake::intakeOff, intake));
        return pCommandGroup;
      }

    public SequentialCommandGroup doAutoFirstShot(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new SpeakerShootCommand(shooter));
        pCommandGroup.addCommands(Commands.waitSeconds(1));
        pCommandGroup.addCommands(Commands.runOnce(indexer::shoot, indexer));
        pCommandGroup.addCommands(Commands.waitSeconds(.3));
        pCommandGroup.addCommands(new StopShooterCommand(shooter));
        return pCommandGroup;
    }

    public SequentialCommandGroup doAutoShot(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(Commands.runOnce(indexer::shoot, indexer));
        pCommandGroup.addCommands(Commands.waitSeconds(.3));
        pCommandGroup.addCommands(new StopShooterCommand(shooter));
        return pCommandGroup;
    }

    public SequentialCommandGroup doSimpleBackUp(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new RelativeDiseredDriveNoPID(5, 0,0, sDrivetrain));
        return pCommandGroup;
      }
    
    public SequentialCommandGroup doSimpleReturnHome(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new SpeakerShootCommand(shooter));
        pCommandGroup.addCommands(new AbsoluteDiseredDriveNoPID(0, 0,0, sDrivetrain));
        return pCommandGroup;
      }

    public SequentialCommandGroup GoToNearSourceNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new AbsoluteDiseredDriveNoPID(6, 4.75,0, sDrivetrain));
        pCommandGroup.addCommands(new AbsoluteDiseredDriveNoPID(6.75, 4.75, sDrivetrain));
        pCommandGroup.addCommands(Commands.waitSeconds(.3));
        return pCommandGroup;
      }
    public SequentialCommandGroup GoToNearMiddleNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new AbsoluteDiseredDriveNoPID(6.75, 0,0, sDrivetrain));
        pCommandGroup.addCommands(Commands.waitSeconds(.3));
        return pCommandGroup;
      }
    public SequentialCommandGroup GoToNearAmpNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new AbsoluteDiseredDriveNoPID(6, -4.75,0, sDrivetrain));
        pCommandGroup.addCommands(new AbsoluteDiseredDriveNoPID(6.75, -4.75, sDrivetrain));
        pCommandGroup.addCommands(Commands.waitSeconds(.3));
        return pCommandGroup;
      }
      

    }