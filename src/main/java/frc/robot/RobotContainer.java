// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SemiAutoConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.FindKS;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetDriveTrain;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ThirdIntakeCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.CameraFindNote;
import frc.robot.commands.auto.oldDrive.AbsoluteDiseredDriveNoPID;
import frc.robot.commands.auto.oldDrive.AutoCommandFactory;
import frc.robot.commands.auto.oldDrive.DriveXfeetYfeetDiseredDegreeAngle;
import frc.robot.commands.auto.oldDrive.RelativeDiseredDriveNoPID;
import frc.robot.commands.climbCommand.ClimberSetSpeedCommand;
import frc.robot.commands.climbCommand.DeployClimber;
import frc.robot.commands.climbCommand.HomeClimber;
import frc.robot.commands.climbCommand.WinchClimber;
import frc.robot.commands.semiAuto.CameraDrive;
import frc.robot.commands.semiAuto.SemiAutoFactory;
import frc.robot.commands.shootCommands.AmpShootCommand;
import frc.robot.commands.shootCommands.SpeakerShootCommand;
import frc.robot.commands.shootCommands.SpeedShoot;
import frc.robot.commands.userNotify.VibrateController;
import frc.robot.sensors.FirstBeamBreak;
import frc.robot.sensors.HopperBeamBreak;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.ServoTest;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ThirdIntakeWheels;
import frc.robot.subsystems.Climber.ClimberSelector;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gameOperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  AutoCommandFactory autoCommandFactory;
  SemiAutoFactory semiAutoFactory;

  public final HopperBeamBreak hopperBeamBreak = new HopperBeamBreak();
  public final FirstBeamBreak firstBeamBreak = new FirstBeamBreak();


  public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(m_driverController);
  public final Intake intake = new Intake(swerveDrivetrain, hopperBeamBreak);
  public final ThirdIntakeWheels thirdIntakeWheels = new ThirdIntakeWheels();
  public final PIDShooter shooter = new PIDShooter();
  public final Indexer indexer = new Indexer();
  public final Climber climber = new Climber();
  public final LimeLightSub noteFinderLimeLight = new LimeLightSub("limelight-pickup", 0);
  public final LimeLightSub shooterLimeLight = new LimeLightSub("limelight-shooter", 20);



  public CameraDrive autoCameraDriveToNoteCommand = new CameraDrive(swerveDrivetrain, noteFinderLimeLight, Constants.SemiAutoConstants.note, intake, this.firstBeamBreak);
  //public IntakeCommand autoIntakeCommand = new IntakeCommand(intake, 0.8);
  public Command autoFindNoteClockWiseCommand = new CameraFindNote(swerveDrivetrain, noteFinderLimeLight, 1);
  public Command autoFindNoteCounterClockWiseCommand = new CameraFindNote(swerveDrivetrain, noteFinderLimeLight, -1);

  public RobotContainer() {
    // Configure the trigger bindings
    semiAutoFactory = new SemiAutoFactory(this);
    autoCommandFactory = new AutoCommandFactory(swerveDrivetrain, indexer, intake, shooter,noteFinderLimeLight, semiAutoFactory, thirdIntakeWheels);
    configureBindings();
    SmartDashboard.putData(new PowerDistribution(1, ModuleType.kRev));
    SmartDashboard.putData(CommandScheduler.getInstance());

    new Trigger(intake::haveNote).onTrue(new VibrateController(m_driverController, 1, .5));
    new Trigger(intake::haveNote).onTrue(new VibrateController(m_gameOperatorController, 1, .5));
    new Trigger(firstBeamBreak::get).onTrue(new VibrateController(m_driverController, 1, 1));



    swerveDrivetrain.setDefaultCommand(new Drive(swerveDrivetrain, () -> -m_driverController.getLeftX(), () -> -m_driverController.getRightX(), () -> -m_driverController.getLeftY()));


    // path planner named commands
    NamedCommands.registerCommand("autoCameraDriveToNote", autoCameraDriveToNoteCommand);
    NamedCommands.registerCommand("autoShot", autoCommandFactory.doAutoShot());
    NamedCommands.registerCommand("autoIntake", runIntakeUntilNote());
    NamedCommands.registerCommand("autoFindNoteClockWise", autoFindNoteClockWiseCommand);
    NamedCommands.registerCommand("autoFindNoteCounterClockWise", autoFindNoteCounterClockWiseCommand);
    NamedCommands.registerCommand("autoSpeakerLineUp", new CameraDrive(swerveDrivetrain, shooterLimeLight, SemiAutoConstants.speaker, this.intake, this.firstBeamBreak));


  }



  private ClimberSelector selectClimberCommand() {

    //boolean isLeftPulled = m_gameOperatorController.axisGreaterThan(2, .6).getAsBoolean();
    //boolean isRightPulled = m_gameOperatorController.axisGreaterThan(3, 0.6).getAsBoolean();

    boolean isLeftPulled = (m_gameOperatorController.getHID().getLeftTriggerAxis()> .6 );
    boolean isRightPulled = (m_gameOperatorController.getHID().getRightTriggerAxis() > .6);

    if ((isLeftPulled && isRightPulled) || (!isLeftPulled && !isRightPulled)) {
      return  ClimberSelector.BOTH;
    }
    if (isLeftPulled) {
      return ClimberSelector.LEFT;
    }
    if (isRightPulled) {
      return ClimberSelector.RIGHT;
    }
    return ClimberSelector.BOTH;
  }

  /*
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Trigger gameOpA = m_gameOperatorController.a();
    Trigger gameOpB = m_gameOperatorController.b();
    Trigger gameOpX = m_gameOperatorController.x();
    Trigger gameOpY = m_gameOperatorController.y();
    Trigger gameOpLeftBumper = m_gameOperatorController.leftBumper();
    Trigger gameOpRightBumper = m_gameOperatorController.rightBumper();
    Trigger gameOpPOVUp = m_gameOperatorController.povUp();
    Trigger gameOpPOVDown = m_gameOperatorController.povDown();
    Trigger gameOpPOVRight = m_gameOperatorController.povRight();
    @SuppressWarnings("unused") // used in climber choice
    Trigger gameOpleftTrigger = m_gameOperatorController.axisGreaterThan(Constants.OperatorConstants.leftTriggerAxis,.6);
     @SuppressWarnings("unused") //used in climber choice
    Trigger gameOpRightTrigger = m_gameOperatorController.axisGreaterThan(Constants.OperatorConstants.rightTriggerAxis,.6);
    Trigger gameOpBackLeft = m_gameOperatorController.button(Constants.OperatorConstants.kLeftBackButton);
    Trigger gameOpBackRight = m_gameOperatorController.button(Constants.OperatorConstants.kRightBackButton);





    Trigger driverBackLeft = m_driverController.button(Constants.OperatorConstants.kLeftBackButton);
    Trigger driverBackRight = m_driverController.button(Constants.OperatorConstants.kRightBackButton);
    Trigger driverLeftBumper = m_driverController.leftBumper();
    Trigger driverRightBumper = m_driverController.rightBumper();
    Trigger driverY = m_driverController.y();

    // Driver mapping

    Trigger driverComtrollerAutoPickupButton = m_driverController.a();
    Trigger driverToggleFieldOriented = driverY;
    Trigger resetDriverEncoder = driverBackRight;
    Trigger resetDriverGyro = driverBackLeft;
    Trigger slowMode = driverLeftBumper;
    Trigger fastMode = driverRightBumper;
    Trigger reduceSpeed = m_driverController.axisGreaterThan(Constants.OperatorConstants.leftTriggerAxis,.6);
    Trigger increaseSpeed = m_driverController.axisGreaterThan(Constants.OperatorConstants.rightTriggerAxis,.6);

    Trigger homeClimber = gameOpX;
    Trigger unused1 = gameOpPOVRight;

    // Operator mapping
    Trigger reverseIntake = gameOpB;
    Trigger intake = gameOpA;
    Trigger speakerShooter = gameOpLeftBumper;
    Trigger ampShooter = gameOpRightBumper;
    Trigger tuneShooter = gameOpBackLeft;
    Trigger shootButton = gameOpY;
    Trigger climberDown = gameOpPOVDown;
    Trigger climberUp = gameOpPOVUp;
    Trigger testButton = gameOpBackRight;

    // todo add unused buttons for driver

    // Operator Commands
    reverseIntake.and(driverComtrollerAutoPickupButton.negate()).whileTrue(backwardIntakeCommand());
    intake.and(driverComtrollerAutoPickupButton.negate()).whileTrue(forwardIntakeCommand());

    speakerShooter.whileTrue(Commands.runOnce(shooter::prepareForSpeaker, shooter));
    ampShooter.whileTrue(Commands.runOnce(shooter::prepareForAmp,shooter));
    tuneShooter.whileTrue(Commands.runOnce(shooter::prepareForTune, shooter));
    speakerShooter.or(ampShooter).or(tuneShooter).whileFalse(Commands.runOnce(shooter::off, shooter));
    shootButton.debounce(.1).and(speakerShooter.or(ampShooter).or(tuneShooter)).onTrue(Commands.runOnce(indexer::shoot, indexer));
    homeClimber.whileTrue(new HomeClimber(climber));
    testButton.whileTrue(autoCommandFactory.doAutoShot());

    climberDown.whileTrue( 
        new SelectCommand<>(
          Map.ofEntries(
            Map.entry(ClimberSelector.BOTH, new WinchClimber(climber, ClimberSelector.BOTH)),
            Map.entry(ClimberSelector.LEFT, new WinchClimber(climber, ClimberSelector.LEFT)),
            Map.entry(ClimberSelector.RIGHT, new WinchClimber(climber, ClimberSelector.RIGHT))),
            this::selectClimberCommand));

    climberUp.whileTrue( 
        new SelectCommand<>(
          Map.ofEntries(
            Map.entry(ClimberSelector.BOTH, new DeployClimber(climber, ClimberSelector.BOTH)),
            Map.entry(ClimberSelector.LEFT, new DeployClimber(climber, ClimberSelector.LEFT)),
            Map.entry(ClimberSelector.RIGHT, new DeployClimber(climber, ClimberSelector.RIGHT))),
            this::selectClimberCommand));

   
    //Driver Commands
    resetDriverEncoder.whileTrue(Commands.runOnce(swerveDrivetrain::initialize, swerveDrivetrain));

    driverToggleFieldOriented.onTrue(Commands.runOnce(swerveDrivetrain::doRelative));
    driverToggleFieldOriented.onFalse(Commands.runOnce(swerveDrivetrain::doAbsolute));
    driverComtrollerAutoPickupButton.whileTrue(semiAutoFactory.autoPickupNote());
    resetDriverGyro.whileTrue(new ResetGyro(swerveDrivetrain));
    slowMode.onTrue(Commands.runOnce(swerveDrivetrain.governor::setSlowMode, swerveDrivetrain));
    fastMode.onTrue(Commands.runOnce(swerveDrivetrain.governor::setFastMode, swerveDrivetrain));
    reduceSpeed.onTrue(Commands.runOnce(swerveDrivetrain.governor::decrement, swerveDrivetrain));
    increaseSpeed.onTrue(Commands.runOnce(swerveDrivetrain.governor::increment, swerveDrivetrain));

    // testSemiAutoShot.whileTrue(new CameraDrive(swerveDrivetrain, shooterLimeLight, SemiAutoConstants.speaker, this.intake, this.firstBeamBreak).
    //                             andThen(autoCommandFactory.doAutoShot())
    //                             .andThen(new CameraFindNote(swerveDrivetrain, noteFinderLimeLight, -1)).andThen(semiAutoFactory.autoPickupNote()));
    // //testSemiAutoShot.whileTrue(new CameraFindNote(swerveDrivetrain, noteFinderLimeLight, 1));
  }
  public Command getTestAuto(){
    Command command = new CameraDrive(swerveDrivetrain, shooterLimeLight, SemiAutoConstants.speaker, this.intake, this.firstBeamBreak);
   // command.andThen(autoCommandFactory.doAutoShot());
    return command;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     //PathPlannerAuto auto = new PathPlannerAuto("StageNoteAuto");
     //PathPlannerAuto auto = new PathPlannerAuto("New Auto");
     //PathPlannerAuto auto = new PathPlannerAuto("OneNote");
     //PathPlannerAuto auto = new PathPlannerAuto("Far Auto"); //todo change to do multiple auto
     //return autoCommandFactory.AlwaysDo().andThen(auto);
    // An example command will be run in autonomous
  return  autoCommandFactory.getAutoCommand();
  }

  public AutoCommandFactory getAutoCommandFactory() {
    return autoCommandFactory;
  }


  public Command forwardIntakeCommand() {
    ParallelCommandGroup commandGroup = new ParallelCommandGroup(new IntakeCommand(intake,.8), new ThirdIntakeCommand(thirdIntakeWheels, RobotConstants.thridIntakeSpeed));
    return commandGroup;
  }
  public Command backwardIntakeCommand() {
    ParallelCommandGroup commandGroup = new ParallelCommandGroup(new IntakeCommand(intake,-.5), new ThirdIntakeCommand(thirdIntakeWheels, -RobotConstants.thridIntakeSpeed));
    return commandGroup;
  }
  public Command runIntakeUntilNote() {
    //example
    Command c = new ParallelDeadlineGroup(new IntakeCommand(intake,.8).until(hopperBeamBreak::getDebouncedBeamBreak), new ThirdIntakeCommand(thirdIntakeWheels, RobotConstants.thridIntakeSpeed));
// beambreak
    return c;
  }

}

