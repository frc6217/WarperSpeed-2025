// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FindKS;
import frc.robot.commands.ResetDriveTrain;
import frc.robot.commands.UserControllerSwitch;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private final CommandJoystick mJoystick = new CommandJoystick(0);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  //public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(mJoystick,m_driverController);
  public final SwerveDrivetrain swerveDrivetrain = new SwerveDrivetrain(mJoystick, m_driverController);
  public final Intake intake = new Intake();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public final Shooter shooter=new Shooter();
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  
    SmartDashboard. putData(new UserControllerSwitch(swerveDrivetrain));
    
    //SmartDashboard.putData("Reset Drive System", new ResetDriveTrain(swerveDrivetrain));
    m_driverController.a().onTrue(Commands.runOnce(intake::on1Intake, intake));
    m_driverController.b().onTrue(Commands.runOnce(intake::off1Intake, intake));
    m_driverController.x().onTrue(Commands.runOnce(intake::on2Intake, intake));
    m_driverController.y().onTrue(Commands.runOnce(intake::off2Intake, intake));
     m_driverController.rightBumper().onTrue(Commands.runOnce(shooter::on, shooter));
    m_driverController.leftBumper().onTrue(Commands.runOnce(shooter::off, shooter));
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
   // return Autos.exampleAuto(m_exampleSubsystem);
   return null;
  }
}
