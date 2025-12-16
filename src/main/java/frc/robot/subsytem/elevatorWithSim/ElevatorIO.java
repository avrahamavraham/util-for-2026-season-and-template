// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.ElevatorWithSim;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface ElevatorIO {
    public static class ElevatorInputs implements LoggableInputs{
        public double Height = 0;
        public double speed = 0;
        public boolean isMotorConnect = false;
        public boolean isClosed = false;
        public double voltage = 0;
        @Override
        public void toLog(LogTable table) {
            table.put("Height", Height);
            table.put("Speed", speed);
            table.put("IsMotorConnect", isMotorConnect);
            table.put("IsClosed", isClosed);
            table.put("Voltage", voltage);
        }

        @Override
        public void fromLog(LogTable table) {
            Height = table.get("Height", Height);
            speed = table.get("Speed", speed);
            isMotorConnect = table.get("IsMotorConnect", isMotorConnect);
            isClosed = table.get("IsClosed", isClosed);
            voltage = table.get("Voltage", voltage);
        }

        public ElevatorInputs clone() {
            ElevatorInputs copy = new ElevatorInputs();
            copy.Height = this.Height;
            copy.speed = this.speed;
            copy.isMotorConnect = this.isMotorConnect;
            copy.isClosed = this.isClosed;
            copy.voltage = this.voltage;
            return copy;
        }
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
