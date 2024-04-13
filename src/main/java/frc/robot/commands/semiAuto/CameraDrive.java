// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiAuto;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.semiAuto.SemiAutoParameters.TARGET;
import frc.robot.sensors.FirstBeamBreak;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;

public class CameraDrive extends Command {
  SwerveDrivetrain drivetrain;
  LimeLightSub ll;
  Intake intake;
  SemiAutoParameters parameters;
  PIDController translationPidController;
  PIDController rotationPidController;
  PIDController strafePidController;
  FirstBeamBreak firstBeamBreak;

  DoubleSupplier translationErrorSupplier;
  DoubleSupplier rotationErrorSupplier;
  DoubleSupplier strafeErrorsSupplier;

  Debouncer debouncer = new Debouncer(0.2);

  /** Creates a new CameraDrive. */
  public CameraDrive(SwerveDrivetrain drivetrain, LimeLightSub ll, SemiAutoParameters parameters, Intake intake, FirstBeamBreak firstBeamBreak) {
    this.drivetrain = drivetrain;
    this.ll = ll;
    this.intake = intake;
    this.parameters = parameters;
    this.firstBeamBreak = firstBeamBreak;
    addRequirements(drivetrain);
    translationPidController =  new PIDController(parameters.translationPID.P, parameters.translationPID.I, parameters.translationPID.D);
    rotationPidController =  new PIDController(parameters.rotationPID.P, parameters.rotationPID.I, parameters.rotationPID.D);
    strafePidController =  new PIDController(parameters.strafePID.P, parameters.strafePID.I, parameters.strafePID.D);

    
    translationPidController.setTolerance(parameters.translationPID.tolerance);
    translationPidController.setSetpoint(parameters.translationPID.setPoint);

    rotationPidController.setTolerance(parameters.rotationPID.tolerance);
    rotationPidController.setSetpoint(parameters.rotationPID.setPoint);

    strafePidController.setTolerance(parameters.strafePID.tolerance);
    strafePidController.setSetpoint(parameters.strafePID.setPoint);

    SmartDashboard.putNumber("CameraDrive P Translation", parameters.translationPID.P);
    SmartDashboard.putNumber("CameraDrive Tralation Setpoint", parameters.translationPID.setPoint);
    SmartDashboard.putNumber("CameraDrive I Translation", parameters.translationPID.I);
    SmartDashboard.putNumber("CameraDrive D Translation", parameters.translationPID.D);
    
    SmartDashboard.putNumber("CameraDrive P rotation", parameters.rotationPID.P);
    SmartDashboard.putNumber("CameraDrive Rotation Setpoint", parameters.rotationPID.setPoint);
    SmartDashboard.putNumber("CameraDrive I rotation", parameters.rotationPID.I);
    SmartDashboard.putNumber("CameraDrive D rotation", parameters.rotationPID.D);

    SmartDashboard.putNumber("CameraDrive P strafe", parameters.strafePID.P);
    SmartDashboard.putNumber("CameraDrive Strafe Setpoint", parameters.strafePID.setPoint);
    SmartDashboard.putNumber("CameraDrive I strafe", parameters.strafePID.I);
    SmartDashboard.putNumber("CameraDrive D strafe", parameters.translationPID.D);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("cameraDriveState", "init");
    //set which pipeline we want
    switch (parameters.target) {
      case NOTE:
        ll.setPipeline(0);
        translationErrorSupplier = ll::getY;
        rotationErrorSupplier = ll::getX;
        strafeErrorsSupplier = ll::getX;
      break;
      case AMP:
      if (drivetrain.allianceSelector.getAllianceColor() == Alliance.Red) {
          ll.setPipeline(2);
        } else {
          ll.setPipeline(3);
        }
        translationErrorSupplier = ll::getY;
        rotationErrorSupplier = ll::getX;
        strafeErrorsSupplier = ll::getX;
        break;
      case SPEAKER:
        if (drivetrain.allianceSelector.getAllianceColor() == Alliance.Red) {
          ll.setPipeline(0);
        } else {
          ll.setPipeline(1);
        }
        translationErrorSupplier = ll::getY;
        rotationErrorSupplier = ll::getSkew;
        strafeErrorsSupplier = ll::getX;
        break;
      case TRAP:
        if (drivetrain.allianceSelector.getAllianceColor() == Alliance.Red) {
          ll.setPipeline(4);
        } else {
          ll.setPipeline(5);
        }
        translationErrorSupplier = ll::getY;
        rotationErrorSupplier = ll::getX;
        strafeErrorsSupplier = ll::getX;
        break;
      case SOURCE:
        if (drivetrain.allianceSelector.getAllianceColor() == Alliance.Red) {
          ll.setPipeline(6);
        } else {
          ll.setPipeline(7);
        }
        translationErrorSupplier = ll::getY;
        rotationErrorSupplier = ll::getX;
        strafeErrorsSupplier = ll::getX;
        break;
      default:
        break;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //
    parameters.translationPID.P = SmartDashboard.getNumber("CameraDrive P Translation", parameters.translationPID.P);
    parameters.translationPID.setPoint = SmartDashboard.getNumber("CameraDrive Tralation Setpoint", parameters.translationPID.setPoint);
    parameters.translationPID.I = SmartDashboard.getNumber("CameraDrive I Translation", parameters.translationPID.I);
    parameters.translationPID.D = SmartDashboard.getNumber("CameraDrive D Translation", parameters.translationPID.D);
    
    parameters.rotationPID.P = SmartDashboard.getNumber("CameraDrive P rotation", parameters.rotationPID.P);
    parameters.rotationPID.setPoint = SmartDashboard.getNumber("CameraDrive Rotation Setpoint", parameters.rotationPID.setPoint);
    parameters.rotationPID.I = SmartDashboard.getNumber("CameraDrive I rotation", parameters.rotationPID.I);
    parameters.rotationPID.D = SmartDashboard.getNumber("CameraDrive D rotation", parameters.rotationPID.D);

    parameters.strafePID.P = SmartDashboard.getNumber("CameraDrive P strafe", parameters.strafePID.P);
    parameters.strafePID.setPoint = SmartDashboard.getNumber("CameraDrive Strafe Setpoint", parameters.strafePID.setPoint);
    parameters.strafePID.I = SmartDashboard.getNumber("CameraDrive I strafe", parameters.strafePID.I);
    parameters.strafePID.D = SmartDashboard.getNumber("CameraDrive D strafe", parameters.translationPID.D);
        

    if (ll.isValid()) {

      double translationAmount = translationPidController.calculate(translationErrorSupplier.getAsDouble());
      translationAmount = MathUtil.clamp(translationAmount, -.5, .5); //todo move constants

      double rotationAmount = rotationPidController.calculate(rotationErrorSupplier.getAsDouble());
      rotationAmount = MathUtil.clamp(rotationAmount, -.5, .5); //todo move constants
       
      
      double strafeAmount = strafePidController.calculate(strafeErrorsSupplier.getAsDouble());//todo test strafe vs rotation for X
      strafeAmount = MathUtil.clamp(strafeAmount, -.5, .5); //todo move constants

      drivetrain.relativeDrive(new Translation2d(-translationAmount, strafeAmount*parameters.direction).times(Constants.RobotConstants.driveMaxVelo), rotationAmount*Constants.RobotConstants.rotationMaxAngleVelo);
    } else {
      drivetrain.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(parameters.target == TARGET.NOTE){
      return firstBeamBreak.getDebouncedBeamBreak();
    }
    return translationPidController.atSetpoint() && rotationPidController.atSetpoint() && strafePidController.atSetpoint();
  }
}
