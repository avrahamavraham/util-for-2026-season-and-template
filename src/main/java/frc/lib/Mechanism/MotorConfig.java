// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Mechanism;

import java.util.function.BiConsumer;
import java.util.function.Consumer;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class MotorConfig {
    private int id;
    private String CanBusName;
    private String MotorName;
    private double Kp;
    private double Ki;
    private double Kd;
    private double Kg;
    private double Ks;
    private double Kv;
    private double Ka;
    private BiConsumer<String,Double> logger;

}
