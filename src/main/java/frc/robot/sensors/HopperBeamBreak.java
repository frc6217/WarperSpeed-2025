// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RobotConstants;

/** Add your docs here. */

public class HopperBeamBreak {
DigitalInput beamNoteDetector =  new DigitalInput(RobotConstants.beamNoteDetectorChannel);

public boolean get() {
    return !beamNoteDetector.get();
}

}
