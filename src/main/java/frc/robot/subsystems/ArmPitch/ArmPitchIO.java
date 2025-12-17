// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmPitch;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmPitchIO {
    @AutoLog
    public class ArmPithcInputs {
        public double position = 0;
        public double velocity = 0;
        public double voltage = 0;
        public boolean isMotorConnect = false;
    }
    /** update the inputs of the Arm */
    public default void updateInputs(ArmPithcInputs inputs) {}
    /** set the angle of the motor */
    public default void SetAngle(double degree) {}
    /** set the voltz of the motor */
    public default void setVolts(double volts) {}
    /** set the pos on the motor */
    public default void setMotorPos(double pos) {}
    /** update the pid and feedfroword from the network table numbers */
    public default void appliePIDF() {}
}
