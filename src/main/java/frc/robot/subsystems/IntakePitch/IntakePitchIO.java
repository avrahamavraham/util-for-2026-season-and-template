// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePitch;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakePitchIO {
    @AutoLog
    public class IntkaePitchInputs {
        public double position = IntakePitchConstants.startingPosition;
        public double velocity = 0;
        public double voltage = 0;
        public boolean isMotorConncted = false;
        public boolean haveGamePiece = false;
    }
    /** update the inputs of the mec */
    public default void updateInputs(IntkaePitchInputs inputs) {}
    /** move the mec to a position */
    public default void setAngle(double angle) {}
    /** move the mec in volts */
    public default void setVolts(double votls) {}
    /** set the motor pos to this pos */
    public default void setMotorPos(double pos) {}
    /** update the pid and feedforword */
    public default void updatePIDF() {}
}
