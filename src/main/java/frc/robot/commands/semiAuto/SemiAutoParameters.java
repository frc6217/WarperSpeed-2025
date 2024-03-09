// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.semiAuto;

/** Add your docs here. */
public class SemiAutoParameters {
    public PIDParameters translationPID;
    public PIDParameters rotationPID;
    public PIDParameters strafePID;
    public double direction;

    public enum TARGET {
        SPEAKER, AMP, NOTE;
    }

    public TARGET target;

    public SemiAutoParameters(TARGET target, PIDParameters translationPID, PIDParameters rotationPID, PIDParameters strafePID, double direction) {
        this.translationPID = translationPID;
        this.rotationPID = rotationPID;
        this.strafePID = strafePID;
        this.direction = direction;
        this.target = target;
    }

    public static class PIDParameters {
        public double P,I,D;
        public double setPoint;
        public double tolerance;

        public PIDParameters(double P, double I, double D, double setPoint, double tolerance) {
            this.P = P;
            this.I = I;
            this.D = D;
            this.setPoint = setPoint;
            this.tolerance = tolerance;
        }
    }
}
