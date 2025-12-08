// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.elevatorWithSim;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.Slot2Configs;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        public Distance Height = Meter.of(0);
        public LinearVelocity speed = MetersPerSecond.of(0);
        public boolean isMotorConnect = false;
        public boolean isClosed = false;
        public Voltage voltage = Volt.of(0);
    }
    /** update the inputs of the elevator */
    public default void updateInputs(ElevatorInputs inputs){}
    /**set the height of the elevator */
    public default void setHeight(Distance Height){}
    /** set the speed of the motors in vlotz*/
    public default void setSpeed(double voltz){}
    /** reaply the pidf constants */
    public default void appliePIDF(){}
}
