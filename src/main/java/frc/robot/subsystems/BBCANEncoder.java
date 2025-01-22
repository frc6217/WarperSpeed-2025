// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.interfaces.IEncoder;
import frc.robot.subsystems.SwerveModule.Constants;

/** Add your docs here. */
public class BBCANEncoder implements IEncoder{
    CANcoder encoder;
    BBCANEncoder(Constants constants){

        //encoder = new CANcoder(constants.absEncoderID, constants.canTypeString);
        encoder = new CANcoder(constants.absEncoderID);
        CANcoderConfiguration caNcoderConfiguration = new CANcoderConfiguration();
        caNcoderConfiguration.MagnetSensor.MagnetOffset = constants.absEncoderOffset;
        caNcoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        caNcoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoder.getConfigurator().apply(caNcoderConfiguration);
        //Offset Is ALways In Degrees
        
        //encoder.configFeedbackCoefficient(0.00153416806, "radians", SensorTimeBase.PerSecond);
    }

    @Override
    public double getRawValue() {
        // TODO Auto-generated method stub
        StatusSignal<Double> value = encoder.getAbsolutePosition();
        //System.out.println("raw incoder value "+value);
        return value.getValueAsDouble();
    }

    @Override
    public Rotation2d getAngle() {
        // TODO Auto-generated method stub
        return Rotation2d.fromRadians(getRawValue()*2*Math.PI);
        //TODO Check this maybe the wrong number see above comment
    }
    
}
