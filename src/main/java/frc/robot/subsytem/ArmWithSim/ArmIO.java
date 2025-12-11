// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.ArmWithSim;

/** Add your docs here. */
public interface ArmIO {
    public class ArmInputs {
        public double position = 0;
        public double velocity = 0;
        public double voltage = 0;
        public boolean isMotorConnect = false;
    }
    /** update the inputs of the Arm */
    public default void updateInputs(ArmInputs inputs){}
    /** set the pos of the motor */
    public default void SetPos(double degree){}
    /** */
    
    
}
