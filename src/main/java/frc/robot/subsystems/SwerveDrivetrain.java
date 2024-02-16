// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.Drive;


public class SwerveDrivetrain extends SubsystemBase {
  /** Creates a new SwerveDrivetrain. */

  public final SwerveModule frontRightModule;// = new SwerveModule(Constants.RobotConstants.frontRight);
  public final SwerveModule backLeftModule;// = new SwerveModule(Constants.RobotConstants.backLeft);
  public final  SwerveModule backRightModule;// = new SwerveModule(Constants.RobotConstants.backRight);
  public final SwerveModule frontLeftModule;// = new SwerveModule(Constants.RobotConstants.frontLeft);

  public final Pigeon2 pigeon2 = new Pigeon2(50,"CTRSwerve");
  public SwerveDriveOdometry sOdometry;
  public SwerveDriveKinematics sKinematics;
  public SwerveModule[] modules;
  public ChassisSpeeds cSpeeds;

  public enum USER_CONTROLLER {
    JOYSTICK,
    XBOX
  }
  public CommandJoystick cj;
  public CommandXboxController cx;
  public USER_CONTROLLER controller = USER_CONTROLLER.JOYSTICK;
  public AllianceSelector allianceSelector;

  public Governor governor = new Governor();
 
  public SwerveDrivetrain(CommandJoystick cj, CommandXboxController cx) {
    this.cj = cj;
    this.cx = cx;
    this.toggleUserController();
    frontRightModule = new SwerveModule(Constants.RobotConstants.frontRight);
    backLeftModule = new SwerveModule(Constants.RobotConstants.backLeft);
    backRightModule = new SwerveModule(Constants.RobotConstants.backRight);
    frontLeftModule = new SwerveModule(Constants.RobotConstants.frontLeft);

    allianceSelector = new AllianceSelector();

    pigeon2.reset();
    
    cSpeeds = new ChassisSpeeds(0, 0, 0);
    sKinematics = new SwerveDriveKinematics(new Translation2d(Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2),
                                            new Translation2d(-Constants.RobotConstants.trackWidth/2, Constants.RobotConstants.trackLength/2),
                                            new Translation2d(-Constants.RobotConstants.trackWidth/2, -Constants.RobotConstants.trackLength/2));
    modules = new SwerveModule[4];
    modules[frontLeftModule.operationOrderID] = frontLeftModule;
    modules[frontRightModule.operationOrderID] = frontRightModule;
    modules[backLeftModule.operationOrderID] = backLeftModule;
    modules[backRightModule.operationOrderID] = backRightModule;
    sOdometry = new SwerveDriveOdometry(sKinematics, getGyroRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d()));
  }

  private Rotation2d getGyroRotation2d(){
    return Rotation2d.fromDegrees(getAngle()); 
  }

  private SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] sPosition = new SwerveModulePosition[4];
    for(SwerveModule module : modules){
      //todo remove when module is back
      //if (module.operationOrderID != Constants.RobotConstants.backLeft.position)
      sPosition[module.operationOrderID] = module.getModulePosition();
    }
    return sPosition;
  }


  public void drive(Translation2d desiredTranslation, double desiredRotation){
    
    cSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredTranslation.getX(), desiredTranslation.getY(), desiredRotation, getGyroRotation2d());
    //SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
  
   // cSpeeds = new ChassisSpeeds(t2D.getX(), t2D.getY() , desiredRotation.getRadians());
    
    // cSpeeds.vxMetersPerSecond = desiredTranslation.getX();
    // cSpeeds.vyMetersPerSecond = desiredTranslation.getY();
    // cSpeeds.omegaRadiansPerSecond = desiredRotation.getRadians();
    
    //cSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(cSpeeds, getGyroRotation2d());
    
    SwerveModuleState[] states = sKinematics.toSwerveModuleStates(cSpeeds);
    SmartDashboard.putNumber("Desired X", desiredTranslation.getX());
    SmartDashboard.putNumber("Desired Y", desiredTranslation.getY());
    SmartDashboard.putNumber("Desired Rotation: (RPS)", desiredRotation);
    SmartDashboard.putNumber("cSpeeds Y", cSpeeds.vyMetersPerSecond);
    
    for(SwerveModule module : modules){
      module.setState(states[module.operationOrderID]);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    sOdometry.update(getGyroRotation2d(), getModulePositions());
    SmartDashboard.putNumber("Gyro Angle", getGyroRotation2d().getDegrees());
   // SmartDashboard.putNumber("Pigeon getYaw Value: ", getAngle());
    SmartDashboard.putNumber("Odometry pose X: ", sOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry pose Y: ", sOdometry.getPoseMeters().getY());

  }

/*
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveTraingo");
    builder.addDoubleProperty("PigeonYaw", this::getPigeonYaw, null);
    builder.addDoubleArrayProperty("Odometry: ", this::getOdometry, null);
    builder.addDoubleArrayProperty("Chassis Speed: ", this::getChassisSpeed, null);
  }

  */

  public Command getResetCommand(){
    return this.runOnce(() -> reset());

  }

  public void enableBrakes(){
    for(SwerveModule module: modules){
      if(module != null)
      //Brakes not working look here
      module.toggleBrakes(true);
    }
  }

  public void unenableBrakes(){
    for(SwerveModule module: modules){
      if(module != null)
      module.toggleBrakes(false);
      }
    }

  public void reset() {
    pigeon2.reset();
    sOdometry.resetPosition(pigeon2.getRotation2d(), getModulePositions(), new Pose2d());
    frontLeftModule.steerEncoder.setPosition(0);
    frontRightModule.steerEncoder.setPosition(0);
    backLeftModule.steerEncoder.setPosition(0);
    backRightModule.steerEncoder.setPosition(0);
    cSpeeds.omegaRadiansPerSecond = 0;
    cSpeeds.vxMetersPerSecond = 0;
    cSpeeds.vyMetersPerSecond = 0;
  }


  public double getAngle() {
      return pigeon2.getYaw().getValueAsDouble();
  }

  public void toggleUserController(){
    if (controller == USER_CONTROLLER.JOYSTICK){
      SmartDashboard.putString("current drive controller", "xbox");
      controller = USER_CONTROLLER.XBOX;
          this.setDefaultCommand(new Drive(this, () -> -cx.getLeftX(), () -> -cx.getRightX(), () -> -cx.getLeftY(), cj));     
    } else if(controller == USER_CONTROLLER.XBOX){
            SmartDashboard.putString("current drive controller", "joystick");
          this.setDefaultCommand(new Drive(this, () -> -cj.getY(), () -> -cj.getZ(), () -> cj.getX(), cj));
      controller = USER_CONTROLLER.JOYSTICK;
    }
  }

  public class Governor{
    public enum MODE {SLOW, FAST};
    public MODE mode = MODE.FAST;
    public double fastInc = Constants.OperatorConstants.kFastInc;
    public double slowInc = Constants.OperatorConstants.kSlowInc;
    public double fastGovernor = Constants.OperatorConstants.kDefaultFast;
    public double slowGovernor = Constants.OperatorConstants.kDefaultSlow;

    public Governor(){
      SmartDashboard.putString("Governor Mode: ", mode.toString());
    }

    public double getGovernor(){
      switch (mode) {
        case SLOW:
          return slowGovernor;
        case FAST:
          return fastGovernor;
      }
      return 0;
    }
    public void setSlowMode(){
      mode = MODE.SLOW;
      SmartDashboard.putString("Governor Mode: ", mode.toString());
    }
    public void setFastMode(){
      mode = MODE.FAST;
      SmartDashboard.putString("Governor Mode: ", mode.toString());
    }
    public void increment(){ 
      switch (mode) {
        case SLOW:
          slowGovernor += slowInc;
        case FAST:
          fastGovernor += fastInc;
      }
    }
    public void decrement(){ 
      switch (mode) {
        case SLOW:
          slowGovernor -= slowInc;
        case FAST:
          fastGovernor -= fastInc;
      }
    }
  }
}
