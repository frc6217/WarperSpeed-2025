// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

/** Add your docs here. */

public class HopperBeamBreak extends SubsystemBase{
    DigitalInput beamNoteDetector =  new DigitalInput(RobotConstants.beamNoteDetectorChannel);
    Debouncer debouncer = new Debouncer(RobotConstants.intakeDebounceTime);
    boolean debouncedBeamBreak = false;


    public boolean get() {
        return !beamNoteDetector.get();
    }

    @Override
    public void periodic() { 
    debouncedBeamBreak = debouncer.calculate(get());
    }

    public boolean getDebouncedBeamBreak() {
        return debouncedBeamBreak;
    }

}

