// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.Gripper;

import frc.lib.util.ITarget;

/** Add your docs here. */
public enum GripperState implements ITarget {
    MOVE(4),
    STOP(0);
    private double velocity;
    private GripperState(double velocity){
        this.velocity = velocity;
    }
    @Override
    public double getTarget(){
        return velocity;
    }
}
