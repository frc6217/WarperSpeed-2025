// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.auto.Intake.AutoIntakeEnd;
import frc.robot.commands.auto.Intake.AutoIntakeStart;
import frc.robot.commands.auto.Shooter.AutoShootEnd;
import frc.robot.commands.auto.Shooter.AutoShootStart;
import frc.robot.commands.semiAuto.SemiAutoFactory;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoCommandFactory {
    SwerveDrivetrain sDrivetrain;
    Indexer indexer;
    Intake intake;
    PIDShooter shooter;
    LimeLightSub limeLightSub;
    SemiAutoFactory semiAutoFactory;
    

    //Potenial Maybe Use
    // public SendableChooser<Command> autoChooserStep1 = new SendableChooser<Command>();
    // public SendableChooser<Command> autoChooserStep2 = new SendableChooser<Command>();
    public SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    SequentialCommandGroup testCommand;

    public AutoCommandFactory(SwerveDrivetrain sDrivetrain, Indexer indexer, Intake intake, PIDShooter shooter, LimeLightSub limeLightSub, SemiAutoFactory semiAutoFactory){
        this.sDrivetrain = sDrivetrain;
        this.indexer = indexer;
        this.shooter = shooter;
        this.limeLightSub = limeLightSub;
        this.intake = intake;
        this.semiAutoFactory = semiAutoFactory;

        testCommand = new SequentialCommandGroup();

        autoChooser.setDefaultOption("No Shot, no move", AlwaysDo().andThen(IntakeOff()).andThen(DoEnd()));
        autoChooser.addOption("No Shot leave from anywhere", AlwaysDo().andThen(getJustBackUp18()).andThen(DoEnd()));

        autoChooser.addOption("Middle, One Shot, No Move ", AlwaysDo().andThen(doSimpleReturnHome()).andThen(doAutoFirstShot()).andThen(DoEnd()));
        autoChooser.addOption("Middle, One Shot, Simple Backup", AlwaysDo().andThen(doSimpleReturnHome()).andThen(doAutoFirstShot()).andThen(doSimpleBackUp()).andThen(DoEnd()));
        autoChooser.addOption("Middle, Two Shot, Score Near Middle Note", AlwaysDo().andThen(doSimpleReturnHome()).andThen(doAutoFirstShot()).andThen(GoToNearMiddleNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).andThen(DoEnd()));

        autoChooser.addOption("Middle, Four Shot, All Near Notes", 
        AlwaysDo().andThen(doSimpleReturnHome()).andThen(doAutoFirstShot()).
        andThen(GoToNearMiddleNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).
        andThen(GoToNearLeftNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).
        andThen(GoToNearRightNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).andThen(DoEnd()));

        autoChooser.addOption("Middle, Three Shot, Not Right Near Note", 
        AlwaysDo().andThen(doSimpleReturnHome()).andThen(doAutoFirstShot()).
        andThen(GoToNearMiddleNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).
        andThen(GoToNearLeftNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).andThen(DoEnd()));

        autoChooser.addOption("Middle, Three Shot, Not left Near Note", 
        AlwaysDo().andThen(doSimpleReturnHome()).andThen(doAutoFirstShot()).
        andThen(GoToNearMiddleNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).
        andThen(GoToNearRightNote()).andThen(doSimpleReturnHome()).andThen(doAutoShot()).andThen(DoEnd()));

        autoChooser.addOption("Left Amp, One Shot, leave to Left", AlwaysDo().andThen(leftSideStart()).andThen(doAutoFirstShot()).andThen(doSimpleBackUp()).andThen(DoEnd()));
        autoChooser.addOption("Left Source, One Shot, leave to Left", AlwaysDo().andThen(leftSideStart()).andThen(doAutoFirstShot()).andThen(doSimpleBackUp()).andThen(DoEnd()));
        autoChooser.addOption("Left Source, Two Shot, get far Note", AlwaysDo().andThen(leftSideStart()).andThen(doAutoFirstShot()).
        andThen(GoToFarLeftFirstNote()).andThen(doAutoShot()).andThen(DoEnd()));
        autoChooser.addOption("Left Source, Three Shot, get far Note", AlwaysDo().andThen(leftSideStart()).
        andThen(GoToFarLeftFirstNote()).andThen(doAutoShot()).
        andThen(GoToFarLeftSecondNote()).andThen(doAutoShot()).andThen(DoEnd()));


        autoChooser.addOption("Right Amp, One Shot, leave to right", AlwaysDo().andThen(rightSideStart()).andThen(doAutoFirstShot()).andThen(rightSideAmpLeave()).andThen(DoEnd()));
        autoChooser.addOption("Right Source, One Shot, leave to right", AlwaysDo().andThen(rightSideStart()).andThen(doAutoFirstShot()).andThen(doSimpleBackUp()).andThen(DoEnd()));
        autoChooser.addOption("RIght Source, Two Shot, get far Note", AlwaysDo().andThen(rightSideStart()).
        andThen(GoToFarRightFirstNote()).andThen(doAutoShot()).andThen(DoEnd()));
        autoChooser.addOption("Left Source, Three Shot, get far Note", AlwaysDo().andThen(leftSideStart()).
        andThen(GoToFarRightFirstNote()).andThen(doAutoShot()).
        andThen(GoToFarRightSecondNote()).andThen(doAutoShot()).andThen(DoEnd()));
         SmartDashboard.putData(autoChooser);

    }
    public Command getAutoCommand(){
      return autoChooser.getSelected();
      }

    public SequentialCommandGroup getJustBackUp18(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new RelativeDiseredDriveNoPID(18, 0, sDrivetrain));
      return pCommandGroup;
    }
    
    public SequentialCommandGroup AlwaysDo(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(Commands.runOnce(sDrivetrain::enableBrakes, sDrivetrain));
        pCommandGroup.addCommands(new SetSwerveOdometry(sDrivetrain, 0, 0, 0));
        return pCommandGroup;
      }
    
    public SequentialCommandGroup leftSideStart(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new SetSwerveOdometry(sDrivetrain, AutoConstants.sideSetupXdistance, AutoConstants.sideSetupYdistance, AutoConstants.sideSetupAngle));
      return pCommandGroup;
    }

    public SequentialCommandGroup leftSideAmpLeave(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(0, 6.5,0, sDrivetrain));
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(18, 6.5,0, sDrivetrain));
      return pCommandGroup;
    }

    public SequentialCommandGroup leftSideSourceLeave(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(0, 12,0, sDrivetrain));
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(18, 12,0, sDrivetrain));
      return pCommandGroup;
    }


    public SequentialCommandGroup rightSideStart(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new SetSwerveOdometry(sDrivetrain, AutoConstants.sideSetupXdistance, -AutoConstants.sideSetupYdistance, AutoConstants.sideRightSetupAngle));
      return pCommandGroup;
    }

     public SequentialCommandGroup rightSideAmpLeave(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(0, -6.5,0, sDrivetrain));
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(18, -6.5,0, sDrivetrain));
      return pCommandGroup;
    }

    public SequentialCommandGroup rightSideSourceLeave(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(0, -12,0, sDrivetrain));
      pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(18, -6.5,0, sDrivetrain));
      return pCommandGroup;
    }


    public SequentialCommandGroup IntakeOff(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new AutoIntakeEnd(intake));
        return pCommandGroup;
      }

    public SequentialCommandGroup doAutoFirstShot(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new AutoShootStart(shooter).andThen(Commands.waitSeconds(.8)).andThen(Commands.runOnce(indexer::shoot, intake)).andThen(Commands.waitSeconds(.4)));
      // pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.7), new AutoShootStart(shooter), Commands.runOnce(indexer::shoot, indexer)));
      pCommandGroup.addCommands(new AutoShootEnd(shooter));
      return pCommandGroup;
    }



    public SequentialCommandGroup doAutoShot(){
      SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
      pCommandGroup.addCommands(new AutoShootStart(shooter).andThen(Commands.waitSeconds(.1)).andThen(Commands.runOnce(indexer::shoot, intake)).andThen(Commands.waitSeconds(.4)));
      // pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.7), new AutoShootStart(shooter), Commands.runOnce(indexer::shoot, indexer)));
      pCommandGroup.addCommands(new AutoShootEnd(shooter));
      return pCommandGroup;
    }
    

    public SequentialCommandGroup doSimpleBackUp(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new RelativeDiseredDriveNoPID(10, 10,0, sDrivetrain));
        return pCommandGroup;
      }
    
    public Command doSimpleReturnHome(){
        Command pCommandGroup = new ParallelDeadlineGroup(
        new DriveXfeetYfeetDiseredDegreeAngle(.5,0,0, sDrivetrain), 
        new SequentialCommandGroup(new ParallelDeadlineGroup(Commands.waitSeconds(0.3), new IntakeCommand(intake, .75)), new AutoShootStart(shooter)));
        return pCommandGroup;
      }

    public SequentialCommandGroup GoToNearLeftNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(1, 0,30, sDrivetrain));
        //pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.nearNoteXdistance , AutoConstants.nearNoteYdistance, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(semiAutoFactory.autoPickupNote());
        pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.3),new IntakeCommand(intake, .75)));
      //  pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.4), new AutoIntakeStart(intake)));
        return pCommandGroup;
      }


    public SequentialCommandGroup GoToNearMiddleNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new ParallelDeadlineGroup( new DriveXfeetYfeetDiseredDegreeAngle(.5, 0,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(semiAutoFactory.autoPickupNote());
        pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.3),new IntakeCommand(intake, .75)));
      //  pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(0), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new AutoIntakeEnd(intake));
        return pCommandGroup;
      }
    public SequentialCommandGroup GoToNearRightNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
       pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(1.5, -1,330, sDrivetrain).withTimeout(.5));
       // pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.nearNoteXdistance , -AutoConstants.nearNoteYdistance, sDrivetrain), new AutoIntakeStart(intake)));
       pCommandGroup.addCommands(semiAutoFactory.autoPickupNote());
       pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.3),new IntakeCommand(intake, .75))); 
       //pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(0), new AutoIntakeStart(intake)));
        return pCommandGroup;
      }

      public SequentialCommandGroup GoToFarLeftFirstNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(6, AutoConstants.farNoteYDistance,0, sDrivetrain));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance - 6, AutoConstants.farNoteYDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(semiAutoFactory.autoPickupNote());
        pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.5), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new AutoIntakeEnd(intake));
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(6, AutoConstants.farNoteYDistance-4,0, sDrivetrain));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.sideSetupXdistance, AutoConstants.sideSetupYdistance, 60 ,sDrivetrain), new AutoShootStart(shooter)));
        return pCommandGroup;
      }

      public SequentialCommandGroup GoToFarLeftSecondNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(2, AutoConstants.farNoteYDistance,0, sDrivetrain));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance - 2, AutoConstants.farNoteYDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance - 2, AutoConstants.farNote2YDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance, AutoConstants.farNote2YDistance,0, sDrivetrain), new AutoIntakeStart(intake)));  
        pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.3), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new AutoIntakeEnd(intake));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance - 2, AutoConstants.farNoteYDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(2, AutoConstants.farNoteYDistance,0, sDrivetrain));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.sideSetupXdistance, AutoConstants.sideSetupYdistance, 300 ,sDrivetrain), new AutoShootStart(shooter)));
        return pCommandGroup;
      }

      public SequentialCommandGroup GoToFarRightFirstNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(5, -AutoConstants.farNoteYDistance,0, sDrivetrain));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance-4, -AutoConstants.farNoteYDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(semiAutoFactory.autoPickupNote());
        pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.5), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(5, -AutoConstants.farNoteYDistance,0, sDrivetrain));
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.sideSetupXdistance, -AutoConstants.sideSetupYdistance, Constants.AutoConstants.sideRightSetupAngle ,sDrivetrain));
        return pCommandGroup;
      }
      
      public SequentialCommandGroup GoToFarRightSecondNote(){
        SequentialCommandGroup pCommandGroup = new SequentialCommandGroup();
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(2, -AutoConstants.farNoteYDistance,0, sDrivetrain));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance - 2, -AutoConstants.farNoteYDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance - 2, -AutoConstants.farNote2YDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance, -AutoConstants.farNote2YDistance,0, sDrivetrain), new AutoIntakeStart(intake)));  
        pCommandGroup.addCommands(new ParallelDeadlineGroup(Commands.waitSeconds(.3), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new AutoIntakeEnd(intake));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.farNoteXdistance - 2, -AutoConstants.farNoteYDistance,0, sDrivetrain), new AutoIntakeStart(intake)));
        pCommandGroup.addCommands(new DriveXfeetYfeetDiseredDegreeAngle(2, -AutoConstants.farNoteYDistance,0, sDrivetrain));
        pCommandGroup.addCommands(new ParallelDeadlineGroup(new DriveXfeetYfeetDiseredDegreeAngle(AutoConstants.sideSetupXdistance, -AutoConstants.sideSetupYdistance, 300 ,sDrivetrain), new AutoShootStart(shooter)));
        return pCommandGroup;
      }

      public ParallelCommandGroup DoEnd(){
        ParallelCommandGroup pCommandGroup = new ParallelCommandGroup();
        pCommandGroup.addCommands(new AutoShootEnd(shooter));
        pCommandGroup.addCommands(new AutoIntakeEnd(intake));
        return pCommandGroup;
      }

    }