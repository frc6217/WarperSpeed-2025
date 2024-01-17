// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.interfaces.IEncoder;

/** Add your docs here. */
public class BBAbsoluteEncoder implements IEncoder{
    SparkAbsoluteEncoder encoder;
    BBAbsoluteEncoder(CANSparkMax motor){
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    }


    @Override
    public double getRawValue() {
        // TODO Auto-generated method stub
        return encoder.getPosition();
    }

    @Override
    public Rotation2d getAngle() {
        // TODO Auto-generated method stub
        return Rotation2d.fromDegrees(encoder.getPosition());
    }

}
