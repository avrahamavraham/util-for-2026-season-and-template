// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.Gripper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface GripperIO {
        public static class GripperInputs implements LoggableInputs{
        public double velocity;
        public double acceleration;
        public double voltage;
        public boolean isMotorConnect = false;
        public boolean haveGamePiece = false;
                @Override
        public void toLog(LogTable table){
            table.put("velocity", velocity);
            table.put("acceleration", acceleration);
            table.put("voltage", voltage);
            table.put("isMotorConnect", isMotorConnect);
            table.put("haveGamePiece", haveGamePiece);
        }
        @Override
        public void fromLog(LogTable table){
            velocity = table.get("velocity", velocity);
            acceleration = table.get("acceleration", acceleration);
            voltage = table.get("voltage", voltage);
            isMotorConnect = table.get("isMotorConnect", isMotorConnect);
            haveGamePiece = table.get("haveGamePiece", haveGamePiece);
        }
    }
    /** update the inputs must happen periodicly */
    public default void updateInputs(GripperInputs inputs){}
    /** set speed in volts */
    public default void setVolts(double volts){}
    /** set speed via velocity voltage */
    public default void setVelocity(double velocity){}
    /** hold the game piece in*/
    public default void hold(){}
}
