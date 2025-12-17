// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmPitch;

import frc.lib.util.ITarget;

/** Add your docs here. */
public enum ArmPitchState implements ITarget {
    up(90),
    down(-90);
    private double angle;

    private ArmPitchState(double angle) {
        this.angle = angle;
    }

    public double getTarget() {
        return angle;
    }
}
