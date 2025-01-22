// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SemiAutoConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.FindKS;
import frc.robot.commands.ResetDriveTrain;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.CameraFindNote;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;

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
 
  public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(m_driverController);
  public final LimeLightSub noteFinderLimeLight = new LimeLightSub("limelight-pickup", 0);

  public Command autoFindNoteClockWiseCommand = new CameraFindNote(swerveDrivetrain, noteFinderLimeLight, -1);
  public Command autoFindNoteCounterClockWiseCommand = new CameraFindNote(swerveDrivetrain, noteFinderLimeLight, 1);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
   // SmartDashboard.putData(new PowerDistribution(1, ModuleType.kRev));
    SmartDashboard.putData(CommandScheduler.getInstance());

    swerveDrivetrain.setDefaultCommand(new Drive(swerveDrivetrain, () -> -m_driverController.getLeftX(), () -> -m_driverController.getRightX(), () -> -m_driverController.getLeftY()));

    // path planner named commands
    NamedCommands.registerCommand("autoFindNoteClockWise", autoFindNoteClockWiseCommand);
    NamedCommands.registerCommand("autoFindNoteCounterClockWise", autoFindNoteCounterClockWiseCommand);
   // NamedCommands.registerCommand("autoSpeakerLineUp", new CameraDrive(swerveDrivetrain, shooterLimeLight, SemiAutoConstants.speaker, this.intake, this.firstBeamBreak));
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
    Trigger driverOpPOVDown = m_driverController.povDown();

    // Driver mapping

    Trigger driverComtrollerAutoPickupButton = m_driverController.a();
    Trigger driverToggleFieldOriented = driverY;
    Trigger resetDriverEncoder = driverBackRight;
    Trigger resetDriverGyro = driverBackLeft;
    Trigger slowMode = driverLeftBumper;
    Trigger fastMode = driverRightBumper;
    Trigger reduceSpeed = m_driverController.axisGreaterThan(Constants.OperatorConstants.leftTriggerAxis,.6);
    Trigger increaseSpeed = m_driverController.axisGreaterThan(Constants.OperatorConstants.rightTriggerAxis,.6);

    Trigger passingShooter = gameOpX;
    Trigger unused1 = gameOpPOVRight;



    // todo add unused buttons for driver

    // Operator Commands

   
    //Driver Commands
    resetDriverEncoder.whileTrue(Commands.runOnce(swerveDrivetrain::initialize, swerveDrivetrain));

    driverToggleFieldOriented.onTrue(Commands.runOnce(swerveDrivetrain::doRelative));
    driverToggleFieldOriented.onFalse(Commands.runOnce(swerveDrivetrain::doAbsolute));
    
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
  // public Command getTestAuto(){
  //   Command command = new CameraDrive(swerveDrivetrain, shooterLimeLight, SemiAutoConstants.speaker, this.intake, this.firstBeamBreak);
  //  // command.andThen(autoCommandFactory.doAutoShot());
  //   return command;
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}

