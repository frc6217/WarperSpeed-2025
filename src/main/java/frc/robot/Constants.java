// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.commands.semiAuto.SemiAutoParameters;
import frc.robot.commands.semiAuto.SemiAutoParameters.PIDParameters;
import frc.robot.commands.semiAuto.SemiAutoParameters.TARGET;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.PIDShooter.ShooterSetPoints;
import frc.robot.subsystems.SwerveModule.Constants.encoderType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kOperatorControllerPort = 2;
    public static final int kLeftBackButton = 7;
    public static final int kRightBackButton = 8;
    public static final int leftTriggerAxis = 2;
    public static final int rightTriggerAxis = 3;
    public static final double kFastInc = 0.1;
    public static final double kSlowInc = 0.01;
    public static final double kDefaultFast = 1;
    public static final double kDefaultSlow = 0.3;
  }

  public static class AutoConstants{
    public static final double nearNoteXdistance = 4;
    public static final double nearNoteYdistance = 2;
    public static final double sideSetupXdistance = -1.48;
    public static final double sideSetupYdistance = 2.5;
    public static final double sideSetupAngle = 60;
    public static final double farNoteXdistance = 24.25;
    public static final double farNoteYDistance = 15.75;
    public static final double farNote2YDistance = 10.25;
  }

  public static class SemiAutoConstants{
    public static final PIDParameters translationPIDNotePickUp = new PIDParameters(0.2, 0, 0, -18.42, .4);
    public static final PIDParameters rotationPIDNotePickUp = new PIDParameters(0.0005, 0, 0, 0, 0.4);
    public static final PIDParameters strafePIDNotePickUp = new PIDParameters(0.01, 0, 0, 0, 1);

    public static final SemiAutoParameters note = new SemiAutoParameters(TARGET.NOTE, translationPIDNotePickUp, rotationPIDNotePickUp, strafePIDNotePickUp, 1);

    public static final PIDParameters translationPIDSpeaker = new PIDParameters(.08, 0, 0, -3.5587, 0.4);
    public static final PIDParameters rotationPIDSpeaker = new PIDParameters(0.0001, 0, 0, 0, 4);
    public static final PIDParameters strafePIDSpeaker = new PIDParameters(0.03, 0, 0, 0, .4);

    public static final SemiAutoParameters speaker = new SemiAutoParameters(TARGET.SPEAKER, translationPIDSpeaker, rotationPIDSpeaker, strafePIDSpeaker, -1);

    public static final PIDParameters translationPIDAmp= new PIDParameters(0.0, 0, 0, 0, 0);
    public static final PIDParameters rotationPIDAmp= new PIDParameters(0.0, 0, 0, 0, 0);
    public static final PIDParameters strafePIDAmp= new PIDParameters(0.0, 0, 0, 0, 0);

    public static final SemiAutoParameters amp = new SemiAutoParameters(TARGET.AMP, translationPIDAmp, rotationPIDAmp, strafePIDAmp, 1);


    public static final ShooterSetPoints speakerSetPoints = new ShooterSetPoints(-4920,-4580);
    public static final ShooterSetPoints ampSetPoints = new ShooterSetPoints(-450,-900);
    public static final ShooterSetPoints trapSetPoints = new ShooterSetPoints(0,0);
    public static final ShooterSetPoints sourceSetPoints = new ShooterSetPoints(0,0);
    public static final ShooterSetPoints idleSetPoints = new ShooterSetPoints(0,0);
    public static final ShooterSetPoints offSetPoints = new ShooterSetPoints(0,0);
  }


  public static class RobotConstants{
    public static final int firstIntakeCanId = 17;
    public static final int secondIntakeCanId =  14;
    public static final int candleCanId = 51;
    public static final int indexerCanId = 7;
    public static final int lowShooterCanId = 5;
    public static final int highShooterCanId = 6;
    public static final double trackWidth = Units.inchesToMeters(24.5);
    public static final double trackLength = Units.inchesToMeters(24.5);
    public static final double driveGearRatio = 6.67;
    public static final double steerGearRatio = 1.2;
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMaxVelo = (5676/60/steerGearRatio)*(wheelDiameter)*Math.PI;
    public static final double rotationMaxAngleVelo = 2*Math.PI*driveMaxVelo;
    public static final int steerMotorCurrentLimit = 40;
    public static final int driveMotorCurrentLimit = 60;
    public static final int shooterMotorCurrentLimit = 60;
    public static final int intakeMotorCurrentLimit = 50;
    public static final int indexerMotorCurrentLimit = 40;
    //Random numberdecided make actually number that the angle velo max actually is. 
/*
    // Practice Robot
    public static final SwerveModule.Constants frontLeft = new SwerveModule.Constants(0, 10, 20, 0, "Front Left", encoderType.Spark);
    public static final SwerveModule.Constants frontRight = new SwerveModule.Constants(1, 12, 22, 0, "Front Right", encoderType.Spark);
    public static final SwerveModule.Constants backLeft = new SwerveModule.Constants(2, 11, 21, 0, "Back Left", encoderType.Spark);
    public static final SwerveModule.Constants backRight = new SwerveModule.Constants(3, 13, 23, 0, "Back Right", encoderType.Spark);
 */
    // Competition Robot

    //bevel gear on wheel goes right
    public static final SwerveModule.Constants frontLeft = new SwerveModule.Constants(0, 16, 15, 60,-0.1062, "Front Left", encoderType.CAN, "CTRSwerve");
    public static final SwerveModule.Constants frontRight = new SwerveModule.Constants(1, 3, 4, 59, -0.1799, "Front Right", encoderType.CAN,"CTRSwerve");
    public static final SwerveModule.Constants backLeft = new SwerveModule.Constants(2, 19, 18, 61,-0.8731, "Back Left", encoderType.CAN, "CTRSwerve");
    public static final SwerveModule.Constants backRight = new SwerveModule.Constants(3, 31, 2, 62,-0.596924, "Back Right", encoderType.CAN,"CTRSwerve");
    public static final double laserNoteThresholdInches = 7.3;
    public static final double indexerStartOffset = .0887;
    public static final double noteDetectorThreshold = 0.000326;
    public static final double highRpmSpeaker = 0;
    public static final double lowRpmSpeaker = 0;
    public static final double highRpmAmp = 0;
    public static final double lowRpmAmp = 0;
    public static final double highRpmTrap = 0;
    public static final double lowRpmTrap = 0;
    public static final double highRpmIdle = 0;
    public static final double lowRpmIdle = 0;
    public static final double shooterVoltageCompensation = 10;
  
  }


}
