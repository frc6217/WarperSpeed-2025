// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.interfaces.IEncoder;
import frc.robot.subsystems.SwerveModule.Constants.encoderType;

public class SwerveModule extends SubsystemBase{
  /** Creates a new SwerveModule. */

  public final CANSparkMax driveMotor;
  public final CANSparkMax steerMotor;
  RelativeEncoder driveEncoder;
  RelativeEncoder steerEncoder;
  SparkPIDController drivePID;

  double driveSetpoint = 0;
  SparkPIDController steerPID;
  double steerSetpoint = 0;
  IEncoder absEncoder;
  String name;
  int operationOrderID;
  double absEncoderOffset;
  SlewRateLimiter slewRate = new SlewRateLimiter(1/RobotConstants.driveSlewTimeInSecond);
  public SwerveModule(Constants constants){
    name = new String(constants.name);

    driveMotor = new CANSparkMax(constants.driveMotorID, MotorType.kBrushless);
    steerMotor = new CANSparkMax(constants.steerMotorID, MotorType.kBrushless);
  //Add to COnstants

    absEncoder = new BBCANEncoder(constants);
    
    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getEncoder();
    drivePID = driveMotor.getPIDController();

    steerPID = steerMotor.getPIDController();


    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    driveMotor.setInverted(false);
    steerMotor.setInverted(false);

    driveMotor.setIdleMode(IdleMode.kCoast);
    steerMotor.setIdleMode(IdleMode.kCoast);

    steerPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    steerPID.setPositionPIDWrappingMinInput(0);
    steerPID.setPositionPIDWrappingEnabled(true);
    steerMotor.setSmartCurrentLimit(RobotConstants.steerMotorCurrentLimit);

    //new swerve
  
  
    // old swerve
    //steerEncoder.setPositionConversionFactor((2 * Math.PI) / 58.3); // gear ratio 58
    
    //steerEncoder.setPosition(0);
    steerEncoder.setPositionConversionFactor(2*Math.PI / 21.4285714286);

    //todo set drive Encoder velocity scale


//todo make drive pid work
    drivePID.setP(0);
    drivePID.setI(0);
    drivePID.setD(0);
    drivePID.setFF(0);

    steerPID.setP(0.3);
    steerPID.setD(0);
    steerPID.setI(0);
    steerPID.setFF(0);

    absEncoderOffset = constants.absEncoderOffset;

    operationOrderID = constants.position;

    //driveMotor.setSmartCurrentLimit(0, 40);
    driveMotor.setSmartCurrentLimit(RobotConstants.driveMotorCurrentLimit);

    

    driveEncoder.setPositionConversionFactor(0.044458);

    steerMotor.burnFlash();
    driveMotor.burnFlash();
  }
  public void initializeEncoder(){
    steerEncoder.setPosition(absEncoder.getAngle().getRadians());
    driveEncoder.setPosition(0);
  }

  /*
  @Override
  public void initSendable(SendableBuilder builder) {
   // builder.setSmartDashboardType(name + "Swerve Module");
    builder.addDoubleProperty(name + "DrivePosition", this::getDrivePosition, null);
    
    builder.addDoubleProperty(name + "DriveSpeed", this::getDriveSpeed, null);
    
    
    builder.addDoubleProperty(name + "SteerSpeed", this::getSteerSpeed, null);
    builder.addDoubleProperty(name + "SteerPosition", this::getSteerPosition, null);
    builder.addDoubleProperty(name + "DriveP", this::getDriveP, this::setDriveP);
    builder.addDoubleProperty(name + "DriveI", this::getDriveI, this::setDriveI);

    builder.addDoubleProperty("DriveD", this::getDriveD, this::setDriveD);
    builder.addDoubleProperty("SteerP", this::getSteerP, this::setSteerP);
    builder.addDoubleProperty("SteerI", this::getSteerI, this::setSteerI);
    builder.addDoubleProperty("SteerD", this::getSteerD, this::setSteerD);
    builder.addDoubleProperty(" AngleState", this::getSetPointAngle, null);
    builder.addDoubleProperty(" SpeedState", this::getSetPointSpeed, null);

  }
  */


  private double getSteerPosition(){
    if(steerEncoder.getPosition() < 0){
      return 2*Math.PI + steerEncoder.getPosition() % (2* Math.PI);
    }else{
      return steerEncoder.getPosition() % (2* Math.PI);
    }
  }

  
  public SwerveModulePosition getModulePosition(){
   // return new SwerveModulePosition(getDistance(), absEncoder.getAngle());
  return new SwerveModulePosition(getDistance(), getCurrentAngle());
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getCurrentAngle());
  }

  private double getDistance() {
    return driveEncoder.getPosition();
  }
  
  private void setSpeed(SwerveModuleState state){
    driveSetpoint = state.speedMetersPerSecond/RobotConstants.driveMaxVelo;
    //SmartDashboard.putNumber(name+" Drive Setpoint ", driveSetpoint);
    if(Math.abs(driveSetpoint) > .001){
     //System.out.println(name  + ": " + MathUtil.clamp(driveSetpoint, -1, 1));
     driveSetpoint = slewRate.calculate(driveSetpoint);
      driveMotor.set(MathUtil.clamp(driveSetpoint, -1, 1));
      //drivePID.setReference(driveSetpoint, ControlType.kVelocity);
    } else{
      driveMotor.set(0);
    }
  }

  private void setAngle(SwerveModuleState state){
    steerSetpoint = state.angle.getRadians();
    if(steerSetpoint < 0){
      steerSetpoint = 2*Math.PI + steerSetpoint % (2* Math.PI);
    }else{
      steerSetpoint = steerSetpoint % (2* Math.PI);
    }
    //steerSetpoint = (Math.abs(steerSetpoint-getSteerPosition()) < 0.15) ? 0 : steerSetpoint;
    steerPID.setReference(steerSetpoint, ControlType.kPosition);
    //System.out.println(name + "pid out " + steerMotor.get());
  }

  public Rotation2d getCurrentAngle(){
    return new Rotation2d(getSteerPosition());
  }

  public void setState(SwerveModuleState state){

    state = SwerveModuleState.optimize(state, getCurrentAngle());
    //state = optimize(state, getCurrentAngle());

    setAngle(state);
    setSpeed(state);

  }

  public void toggleBrakes(boolean toggle){
    if(toggle){
    steerMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setIdleMode(IdleMode.kBrake);
  }
    else{
      driveMotor.setIdleMode(IdleMode.kCoast);
      steerMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  @Override
  public void periodic() {

    // if abs enc value and relative enc val is not within ~5% then log an error
   // if((absEncoder.getAngle().getRadians() - getSteerPosition())/getSteerPosition() >= 0.05) {
     // System.err.print("module:" + name + "is outside of 5%");
  //  }

    // if we aren't adjusting steering AND some time passed (5sec maybe)
    // then get ABS enc value and set it to relative postion (with conversion?)
   // SmartDashboard.putNumber(name + " Wheel Angle (degree)", Math.round(Math.toDegrees(getSteerPosition())));
    //SmartDashboard.putNumber(name + " Absolute Encoder", absEncoder.getAngle().getDegrees());
    //SmartDashboard.putNumber(name + " Absolute Encoder Raw Value: ", absEncoder.getRawValue());
    //SmartDashboard.putNumber(name + " Steer Setpoints", Math.toDegrees(steerSetpoint));
    
    //SmartDashboard.putNumber(name + "RPM of Motor", driveMotor.getEncoder().getVelocity());
    /*
    SmartDashboard.putNumber(name + " DrivePosition", getDrivePosition());
    SmartDashboard.putNumber(name + " DriveSpeed", getDriveSpeed());
    

    SmartDashboard.putNumber(name + " DriveP", getDriveP());
    SmartDashboard.putNumber(name + " DriveI", getDriveI());
    SmartDashboard.putNumber(name + " DriveD", getDriveD());

    SmartDashboard.putNumber(name + " SteerP", getSteerP());
    SmartDashboard.putNumber(name + " SteerI", getSteerI());
    SmartDashboard.putNumber(name + " SteerD", getSteerD());

    SmartDashboard.putNumber(name + " AngleState", getSetPointAngle());
    SmartDashboard.putNumber(name + " SpeedState", getSetPointSpeed());

    SmartDashboard.putNumber(name + " SteerSpeed", getSteerSpeed());
    SmartDashboard.putNumber(name + " SteerPosition", getSteerPosition());

    SmartDashboard.putNumber(name + " abs angle: ", absEncoder.getAngle().getDegrees());
*/
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
   * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
   * support for continuous input.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   */
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle =
        placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  public static class Constants{
    public int position;
    public int driveMotorID;
    public int steerMotorID;
    public int absEncoderID;
    public double absEncoderOffset;
    public enum encoderType {
      CAN,
      Spark
    }
    public String canTypeString;
    public encoderType type;
    public String name;
    public Constants(int position, int driveMotorID, int steerMotorID, int absEncoderID, double absEncoderOffset, String name, encoderType type, String canTypeString) {
      this.position = position;
      this.driveMotorID = driveMotorID;
      this.steerMotorID = steerMotorID;
      this.absEncoderID = absEncoderID;
      this.absEncoderOffset = absEncoderOffset;
      this.canTypeString = canTypeString;
      this.name = name;
      this.type = type;
    }
    
  }

}
