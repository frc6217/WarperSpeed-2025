// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.FindKS;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetDriveTrain;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ThirdIntakeCommand;
import frc.robot.commands.auto.AbsoluteDiseredDriveNoPID;
import frc.robot.commands.auto.AutoCommandFactory;
import frc.robot.commands.auto.DriveXfeetYfeetDiseredDegreeAngle;
import frc.robot.commands.auto.RelativeDiseredDriveNoPID;
import frc.robot.commands.climbCommand.ClimberSetSpeedCommand;
import frc.robot.commands.climbCommand.DeployClimber;
import frc.robot.commands.climbCommand.WinchClimber;
import frc.robot.commands.semiAuto.CameraDrive;
import frc.robot.commands.semiAuto.SemiAutoFactory;
import frc.robot.commands.shootCommands.AmpShootCommand;
import frc.robot.commands.shootCommands.SpeakerShootCommand;
import frc.robot.commands.shootCommands.SpeedShoot;
import frc.robot.commands.userNotify.VibrateController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.PIDShooter;
import frc.robot.subsystems.ServoTest;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.ThirdIntakeWheels;

import java.util.Map;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  // The robot's subsystems and commands are defined here...
 // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

// private final CommandJoystick mJoystick = new CommandJoystick(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_gameOperatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  AutoCommandFactory autoCommandFactory;
  SemiAutoFactory semiAutoFactory;
  //public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(mJoystick,m_driverController);
  public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(m_driverController);
  public final Intake intake = new Intake(swerveDrivetrain);
  public final ThirdIntakeWheels thirdIntakeWheels = new ThirdIntakeWheels();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
 // public final Shooter shooter = new Shooter();
 public final PIDShooter shooter = new PIDShooter();
  public final Indexer indexer = new Indexer();
  public final Climber climber = new Climber();
  public final LimeLightSub noteFinderLimeLight = new LimeLightSub("limelight-pickup", 10);
  public final LimeLightSub shooterLimeLight = new LimeLightSub("limelight-shooter", 0);
  
  //ServoTest servoTest = new ServoTest();

  public RobotContainer() {
    // Configure the trigger bindings
        semiAutoFactory = new SemiAutoFactory(this);
    autoCommandFactory = new AutoCommandFactory(swerveDrivetrain, indexer, intake, shooter,noteFinderLimeLight, semiAutoFactory);
    configureBindings();
    SmartDashboard.putData(new PowerDistribution(1, ModuleType.kRev));
    SmartDashboard.putData(CommandScheduler.getInstance());

    Trigger gameOpA = m_gameOperatorController.a();
    Trigger gameOpB = m_gameOperatorController.b();
    Trigger gameOpX = m_gameOperatorController.x();
    Trigger gameOpY = m_gameOperatorController.y();
    Trigger gameOpLeftBumper = m_gameOperatorController.leftBumper();
    Trigger gameOpRightBumper = m_gameOperatorController.rightBumper();
    Trigger gameOpPOVUp = m_gameOperatorController.povUp();
    Trigger gameOpPOVDown = m_gameOperatorController.povDown();
    Trigger gameOpPOVRight = m_gameOperatorController.povRight();
    Trigger gameOpleftTrigger = m_gameOperatorController.axisGreaterThan(Constants.OperatorConstants.leftTriggerAxis,.6);
    Trigger gameOpRightTrigger = m_gameOperatorController.button(Constants.OperatorConstants.kLeftBackButton);//m_gameOperatorController.axisGreaterThan(Constants.OperatorConstants.rightTriggerAxis,.6);
    Trigger driverBackLeft = m_driverController.button(Constants.OperatorConstants.kLeftBackButton);
    Trigger driverBackRight = m_driverController.button(Constants.OperatorConstants.kRightBackButton);
    Trigger driverLeftBumper = m_driverController.leftBumper();
    Trigger driverRightBumper = m_driverController.rightBumper();
    Trigger driverY = m_driverController.y();
   
   
    Trigger driverComtrollerAutoPickupButton = m_driverController.a();

    //m_driverController.b().whileTrue(Commands.print("start").andThen(new CameraDrive(swerveDrivetrain, shooterLimeLight, Constants.SemiAutoConstants.speaker, intake)).andThen(Commands.print("End")));
    swerveDrivetrain.setDefaultCommand(new Drive(swerveDrivetrain, () -> -m_driverController.getLeftX(), () -> -m_driverController.getRightX(), () -> -m_driverController.getLeftY()));
    driverBackRight.whileTrue(Commands.runOnce(swerveDrivetrain::initialize, swerveDrivetrain));
    //SmartDashboard.putData("Reset Drive System", new ResetDriveTrain(swerveDrivetrain));
    gameOpB.and(driverComtrollerAutoPickupButton.negate()).whileTrue(backwardIntakeCommand());
    gameOpA.and(driverComtrollerAutoPickupButton.negate()).whileTrue(forwardIntakeCommand());
   
    //m_gameOperatorController.button(Constants.OperatorConstants.kLeftBackButton).whileTrue(new SpeedShoot(-.15, -.10, shooter));
    new Trigger(intake::haveNote).onTrue(new VibrateController(m_driverController, 1));
    
    //m_driverController.y().whileTrue(new DriveXfeetYfeetDiseredDegreeAngle(0, 0, 25, swerveDrivetrain));
    //gameOpleftTrigger.whileTrue(new SpeedShoot(.65, .75, shooter));



    gameOpLeftBumper.whileTrue(Commands.runOnce(shooter::prepareForSpeaker, shooter));
    gameOpRightBumper.whileTrue(Commands.runOnce(shooter::prepareForAmp,shooter));
    gameOpRightTrigger.whileTrue(Commands.runOnce(shooter::prepareForTune, shooter));
    gameOpLeftBumper.or(gameOpRightBumper).or(gameOpRightTrigger).whileFalse(Commands.runOnce(shooter::off, shooter));



    gameOpY.and(gameOpLeftBumper.or(gameOpRightBumper).or(gameOpRightTrigger)).onTrue(Commands.runOnce(indexer::shoot, indexer));


    //Climber
   // m_driverController.povDown().onTrue(new VibrateController(m_driverController, 1));

    //m_gameOperatorController.povDownLeft().whileTrue(new ClimberSetSpeedCommand(climber, RobotConstants.climberSpeed, 0));
   // m_gameOperatorController.povUpLeft().whileTrue(new ClimberSetSpeedCommand(climber, -RobotConstants.climberSpeed, 0));
    //m_gameOperatorController.povDownRight().whileTrue(new ClimberSetSpeedCommand(climber, 0, RobotConstants.climberSpeed));
    //m_gameOperatorController.povUpRight().whileTrue(new ClimberSetSpeedCommand(climber, 0, -RobotConstants.climberSpeed));

    //gameOpPOVDown.whileTrue(new WinchClimber(climber);
    gameOpPOVDown.whileTrue( 
        new SelectCommand<>(
          Map.ofEntries(
            Map.entry(CommandSelector.BOTH, new WinchClimber(climber)),
            Map.entry(CommandSelector.LEFT, new ClimberSetSpeedCommand(climber, RobotConstants.climberSpeed, 0)),
            Map.entry(CommandSelector.RIGHT, new ClimberSetSpeedCommand(climber, 0, RobotConstants.climberSpeed))),
            this::selectClimberCommand));

    gameOpPOVUp.whileTrue( 
        new SelectCommand<>(
          Map.ofEntries(
            Map.entry(CommandSelector.BOTH, new DeployClimber(climber)),
            Map.entry(CommandSelector.LEFT, new ClimberSetSpeedCommand(climber, -RobotConstants.climberSpeed, 0)),
            Map.entry(CommandSelector.RIGHT, new ClimberSetSpeedCommand(climber, 0, -RobotConstants.climberSpeed))),
            this::selectClimberCommand));

   // gameOpPOVUp.whileTrue(new DeployClimber(climber));
   
    //Driver controls
    driverY.onTrue(Commands.runOnce(swerveDrivetrain::doRelative));
    driverY.onFalse(Commands.runOnce(swerveDrivetrain::doAbsolute));
    driverComtrollerAutoPickupButton.whileTrue(semiAutoFactory.autoPickupNote());
    driverBackLeft.whileTrue(new ResetGyro(swerveDrivetrain));
    m_driverController.leftBumper().onTrue(Commands.runOnce(swerveDrivetrain.governor::setSlowMode, swerveDrivetrain));
    m_driverController.rightBumper().onTrue(Commands.runOnce(swerveDrivetrain.governor::setFastMode, swerveDrivetrain));
    m_driverController.axisGreaterThan(Constants.OperatorConstants.leftTriggerAxis,.6).onTrue(Commands.runOnce(swerveDrivetrain.governor::decrement, swerveDrivetrain));
    m_driverController.axisGreaterThan(Constants.OperatorConstants.rightTriggerAxis,.6).onTrue(Commands.runOnce(swerveDrivetrain.governor::increment, swerveDrivetrain));
    
  }

  private enum CommandSelector {
    BOTH,
    LEFT,
    RIGHT
  };

  private CommandSelector selectClimberCommand() {

    //boolean isLeftPulled = m_gameOperatorController.axisGreaterThan(2, .6).getAsBoolean();
    //boolean isRightPulled = m_gameOperatorController.axisGreaterThan(3, 0.6).getAsBoolean();

    boolean isLeftPulled = (m_gameOperatorController.getHID().getLeftTriggerAxis()> .6 );
    boolean isRightPulled = (m_gameOperatorController.getHID().getRightTriggerAxis() > .6);

    System.out.println("state: left: " + isLeftPulled + " right: " + isRightPulled);

    if ((isLeftPulled && isRightPulled) || (!isLeftPulled && !isRightPulled)) {
      System.out.println("run both");
      return  CommandSelector.BOTH;
    }
    if (isLeftPulled) {
      return CommandSelector.LEFT;
    }
    if (isRightPulled) {
      return CommandSelector.RIGHT;
    }

      System.out.println("are we here?");
    return CommandSelector.BOTH;
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
   

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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
}
