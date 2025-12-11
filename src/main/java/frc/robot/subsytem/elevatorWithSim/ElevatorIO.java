// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.ElevatorWithSim;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        public double Height = 0;
        public double speed = 0;
        public boolean isMotorConnect = false;
        public boolean isClosed = false;
        public double voltage = 0;
    }
    /** update the inputs of the elevator */
    public default void updateInputs(ElevatorInputs inputs){}
    /**set the height of the elevator */
    public default void setHeight(Double Height){}
    /** set the speed of the motors in vlotz*/
    public default void setSpeed(double voltz){}
    /** reaply the pidf constants */
    public default void appliePIDF(){}
}
