// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsytem.ElevatorWithSim.Elevator;
import frc.robot.subsytem.ElevatorWithSim.ElevatorIOSim;

/** Add your docs here. */
public class Subsytem {
    // GeneralWithSim general = new GeneralWithSim(new GeneralIOSim(SingleJointedArmSim.class.getName()));
    // GeneralWithoutSim general = new GeneralWithoutSim();
    public Elevator elevator = new Elevator(new ElevatorIOSim());
}
