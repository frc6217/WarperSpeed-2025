// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.ResourceBundle.Control;

import javax.swing.text.Position;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialVelocityDutyCycle;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.RobotConstants;
import frc.robot.interfaces.IEncoder;
import frc.robot.subsystems.SwerveModule.Constants.encoderType;

public class SwerveModule extends SubsystemBase{
  /** Creates a new SwerveModule. */

  public final TalonFX driveMotor;
  public final TalonFX steerMotor;
  public PositionDutyCycle steerPID = new PositionDutyCycle(0);
  public VelocityDutyCycle drivePID = new VelocityDutyCycle(0);
  Slot0Configs driveConfigs = new Slot0Configs();
  ClosedLoopRampsConfigs driveRamp = new ClosedLoopRampsConfigs();
  ClosedLoopRampsConfigs steerRamp = new ClosedLoopRampsConfigs();
  double driveSetpoint = 0;
  Slot0Configs steerConfigs = new Slot0Configs();
  double steerSetpoint = 0;
  IEncoder absEncoder;
  String name;
  Timer syncTimer = new Timer();
  int operationOrderID;
  double absEncoderOffset;
  SlewRateLimiter slewRate = new SlewRateLimiter(1/RobotConstants.driveSlewTimeInSecond);
  public SwerveModule(Constants constants){
    syncTimer.start();
    name = new String(constants.name);
    
    driveMotor = new TalonFX(constants.driveMotorID);
    steerMotor = new TalonFX(constants.steerMotorID);


    
  //Add to COnstants
    absEncoder = new BBCANEncoder(constants);
    

    driveMotor.setNeutralMode(NeutralModeValue.Coast);
    steerMotor.setNeutralMode(NeutralModeValue.Coast);

    // steerPID.setPositionPIDWrappingMaxInput(2 * Math.PI);
    // steerPID.setPositionPIDWrappingMinInput(0);
    // steerPID.setPositionPIDWrappingEnabled(true);

    //new swerve
  
  
    // old swerve
    //steerEncoder.setPositionConversionFactor((2 * Math.PI) / 58.3); // gear ratio 58
    
    //steerEncoder.setPosition(0);
    

    //todo set drive Encoder velocity scale

    SmartDashboard.putData(steerMotor);


//todo make drive pid work
    driveRamp.withDutyCycleClosedLoopRampPeriod(.2);
    steerRamp.withDutyCycleClosedLoopRampPeriod(0);

    steerConfigs = steerConfigs.withKA(0).withKD(0).withKG(0).withKI(.6).withKP(3).withKS(0).withKV(0);
    steerMotor.getConfigurator().apply(steerConfigs);
    steerMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(21.4285714).withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));
    steerMotor.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    steerMotor.getConfigurator().apply(steerRamp);
    steerMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(RobotConstants.steerMotorCurrentLimit).withSupplyCurrentLimitEnable(true));
    ClosedLoopGeneralConfigs d = new ClosedLoopGeneralConfigs();
    d.ContinuousWrap = true;
    StatusCode sCode = steerMotor.getConfigurator().apply(d);
    steerPID.withFeedForward(RobotConstants.feedforward);
    steerPID.withSlot(0);

    
   
    driveConfigs = driveConfigs.withKA(0).withKD(0).withKG(0).withKI(0.35).withKP(0.32).withKS(0).withKV(0);
    driveMotor.getConfigurator().apply(driveConfigs);
    driveMotor.getConfigurator().apply(new FeedbackConfigs().withSensorToMechanismRatio(3.13298));
    driveMotor.getConfigurator().apply(driveRamp);
    driveMotor.getConfigurator().apply(new MotorOutputConfigs().withDutyCycleNeutralDeadband(.016));
    driveMotor.getConfigurator().apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(RobotConstants.driveMotorCurrentLimit).withSupplyCurrentLimitEnable(true));
    drivePID.withFeedForward(RobotConstants.feedforward);
    drivePID.withSlot(0);

    steerMotor.setPosition(absEncoder.getAngle().getRotations());

    absEncoderOffset = constants.absEncoderOffset;

    operationOrderID = constants.position;

    //driveMotor.setSmartCurrentLimit(0, 40);

    steerMotor.setControl(steerPID);

  }
  public void initializeEncoder(){
    steerMotor.setPosition(absEncoder.getAngle().getRotations());
    //steerMotor.setPosition(0);
    driveMotor.setPosition(0);


    //steerMotor.setPosition(absEncoder.getAngle().getRotations());
   
  }


  private double getSteerPosition(){
    if(steerMotor.getPosition().getValueAsDouble() < 0){
      return 1 + steerMotor.getPosition().getValueAsDouble() % 1;
    }else{
      return steerMotor.getPosition().getValueAsDouble() % 1;
    }
  }

  
  public SwerveModulePosition getModulePosition(){
   // return new SwerveModulePosition(getDistance(), absEncoder.getAngle());
  return new SwerveModulePosition(getDistance(), getCurrentAngle());
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), getCurrentAngle());
  }

  private double getDistance() {
    return driveMotor.getPosition().getValueAsDouble();
  }
  
  private void setSpeed(SwerveModuleState state){
    driveSetpoint = state.speedMetersPerSecond;
    //SmartDashboard.putNumber(name+" Drive Setpoint ", driveSetpoint);
      drivePID.withVelocity(driveSetpoint);
      driveMotor.setControl(drivePID);
  }

  private void setAngle(SwerveModuleState state){
    steerSetpoint = state.angle.getRotations();
    
    
    //steerSetpoint = (Math.abs(steerSetpoint-getSteerPosition()) < 0.15) ? 0 : steerSetpoint;

    steerPID.withPosition(steerSetpoint);
    steerMotor.setControl(steerPID);
    //System.out.println(name + "pid out " + steerMotor.get());
  }

  public Rotation2d getCurrentAngle(){
    return new Rotation2d(getSteerPosition()*2*Math.PI);
  }

  public void setState(SwerveModuleState state){

    SmartDashboard.putNumber(name + " input angle", state.angle.getRotations());
    SmartDashboard.putNumber(name + " getCurrentAngle", getCurrentAngle().getRotations());

   // state = SwerveModuleState.optimize(state, getCurrentAngle());
    state = optimize(state, getCurrentAngle());

    SmartDashboard.putNumber(name + " Desired Angle", state.angle.getRotations());
    SmartDashboard.putNumber(name + "PID Out ", steerPID.Position);

    //remove me when I wanna drive
  
    setAngle(state);
    setSpeed(state);

  }

  public void toggleBrakes(boolean toggle){
    if(toggle){
    steerMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }
    else{
      driveMotor.setNeutralMode(NeutralModeValue.Coast);
      steerMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }
  public void syncEncoders() {
    steerMotor.setPosition(absEncoder.getAngle().getRotations());
    System.out.println("Did encoder sync");
  }

  @Override
  public void periodic() {
    if(RobotConstants.autoSyncEncoder){
      boolean isMoving = steerMotor.getVelocity().getValueAsDouble() > RobotConstants.syncThreshold;
      boolean isSyncTime = syncTimer.hasElapsed(RobotConstants.autoSyncTimer);
      if((!isMoving) && isSyncTime) {
        syncEncoders();
        syncTimer.restart();
      }
    }
   // SmartDashboard.putNumber(name + " angle", getSteerPosition());
   SmartDashboard.putNumber(name + " abs encoder", absEncoder.getAngle().getRotations());
   SmartDashboard.putNumber(name + "relative encoder raw", steerMotor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber(name + "relative encoder unwrap", this.getSteerPosition());

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
  public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {

    double targetRotation = desiredState.angle.getRadians() + Math.PI; 
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = (targetRotation) - (currentAngle.getRadians());
    SmartDashboard.putNumber(name +" delta", delta);
    SmartDashboard.putNumber(name + " targetRotation", targetRotation);
    SmartDashboard.putNumber(name + " currentRotation", currentAngle.getRadians());
    final double QUARTER_TURN = Math.PI/2;
    if (Math.abs(delta) > QUARTER_TURN) {
      targetSpeed = -targetSpeed;
      targetRotation = delta > QUARTER_TURN ? (targetRotation += Math.PI) : (targetRotation += Math.PI);;

    }
    
    if (this.operationOrderID == 0)
    {
      SmartDashboard.putNumber( name + " optimized target angle", targetRotation);
      SmartDashboard.putNumber( name + " optimized target speed", targetSpeed);
      SmartDashboard.putNumber( name + " optimized target delta", delta);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromRadians(targetRotation));
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
