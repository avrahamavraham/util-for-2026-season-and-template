// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    public class ShooterInputs {
        public double velocity = 0;
        public double acc = 0;
        public boolean isMotorConnected = false;
    }
    /** update the inputs of the mec */
    public default void updateInputs(ShooterInputs inputs){}
    /** set the velocity of the motor in rpm */
    public default void setSpeed(double speed){}
    /** set the speed of the motor with volts */
    public default void setVolts(double volts){}
    /** applid the pidf from the tunning  */
    public default void updatePidf(){}
} 
