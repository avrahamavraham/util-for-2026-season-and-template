// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret.ShooterIO.ShooterInputs;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterInputs inputs;
    /** Creates a new Turret. */
    public Shooter(ShooterIO io) {
        this.io = io;
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public void shoot(){

    }
    /** in rpm */
    public void setVelocity(double velocity){
        io.setSpeed(velocity);
    }
    public void setVolts(double volts){
        io.setVolts(volts);
    }
    public void updatePidf(){
        io.updatePidf();
    }
    /** use this to get the velocity you need */
    // public double WhatVelocity(double distance , double angleOfShooter){
    //     return 
    // }
}
