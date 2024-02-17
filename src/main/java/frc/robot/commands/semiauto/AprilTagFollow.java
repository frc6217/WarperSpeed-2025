// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiauto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSub;
import frc.robot.subsystems.SwerveDrivetrain;

public class AprilTagFollow extends Command {
  /** Creates a new AprilTagFollow. */

  SwerveDrivetrain sDrivetrain;
  LimeLightSub limeLightSub;

  float rotationSetpoint = 0;
  Double strafeSetpoint = .5;
  Double translationSetpoint = .5;

  PIDController rotationPidController = new PIDController(0, 0, 0);
  PIDController strafePidController = new PIDController(0, 0, 0);
  PIDController translationPidController = new PIDController(0, 0, 0);

  public AprilTagFollow(SwerveDrivetrain sDrivetrain, LimeLightSub limeLightSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.limeLightSub = limeLightSub;
    this.sDrivetrain = sDrivetrain;
    addRequirements(limeLightSub, sDrivetrain);

    SmartDashboard.putData(rotationPidController);
    SmartDashboard.putData(strafePidController);
    SmartDashboard.putData(translationPidController);

    SmartDashboard.putNumber("LimeLight p translation", .1);
    SmartDashboard.putNumber("LimeLight p strafe", .1);
    SmartDashboard.putNumber("LimeLight p rotation", .1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPidController.setSetpoint(rotationSetpoint);
    strafePidController.setSetpoint(strafeSetpoint);
    translationPidController.setSetpoint(translationSetpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pTranslation = SmartDashboard.getNumber("LimeLight p translation", 0);
    double pStrafe = SmartDashboard.getNumber("LimeLight p strafe", 0);
    double pRotation = SmartDashboard.getNumber("LimeLight p rotation", 0);

    translationPidController.setP(pTranslation);
    strafePidController.setP(pStrafe);
    rotationPidController.setP(pRotation);

    double getTranslation = -translationPidController.calculate(1/limeLightSub.getArea());
    double getStrafe = strafePidController.calculate(limeLightSub.getX());
    double getRotation = rotationPidController.calculate(limeLightSub.getY());
      
    getTranslation = MathUtil.clamp(getTranslation, -1, 1);
    getStrafe = MathUtil.clamp(getStrafe, -1, 1);
    getRotation = MathUtil.clamp(getRotation, -1, 1);
    
    SmartDashboard.putNumber("translation AprilTag pid out", getTranslation);
    SmartDashboard.putNumber("strafe AprilTag pid out", getStrafe);
    SmartDashboard.putNumber("rotation AprilTag pid out", getRotation);

    if(limeLightSub.getArea() != 0){
      sDrivetrain.drive(new Translation2d(getStrafe, getTranslation), getRotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sDrivetrain.drive(new Translation2d(0,0), 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (
     Math.abs(limeLightSub.getX() - strafeSetpoint) < 1 &&
     Math.abs(limeLightSub.getArea() - translationSetpoint) < 1 && 
     Math.abs(limeLightSub.getY() - rotationSetpoint) < 1
     );
  }
}
