// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO{
    private FlywheelSim Sim;
    private TalonFX m_shooter = new TalonFX(ShooterConstants.m_MotorId,ShooterConstants.m_CanBusName);
    private MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
    public ShooterIOSim(){
        Sim = new FlywheelSim(LinearSystemId.createFlywheelSystem(ShooterConstants.dcMotor, ShooterConstants.JKgMeterSqured, ShooterConstants.gearRatio),
            ShooterConstants.dcMotor);
                TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        FeedbackConfigs feedbackConfigsspin = talonFXConfiguration.Feedback;
        feedbackConfigsspin.SensorToMechanismRatio = ShooterConstants.POSITION_CONVERSION_FACTOR;
        MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = ShooterConstants.NeutralMode;
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterConstants.MotionMagicConstants.MOTION_MAGIC_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ShooterConstants.MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ShooterConstants.MotionMagicConstants.MOTION_MAGIC_JERK;

        talonFXConfiguration.Slot0 = ShooterConstants.MotionMagicConstants.slot0Configs;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_shooter.getConfigurator().apply(talonFXConfiguration);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }
    private void updateSimulation(){
        Sim.setInputVoltage(m_shooter.getSimState().getMotorVoltage());
        Sim.update(0.02);
        m_shooter.getSimState().setRotorAcceleration(Units.radiansToRotations(Sim.getAngularAccelerationRadPerSecSq())* ShooterConstants.POSITION_CONVERSION_FACTOR);
        m_shooter.getSimState().setRotorVelocity((Sim.getAngularVelocityRPM() /60 ) * ShooterConstants.POSITION_CONVERSION_FACTOR);
    }
    @Override
    public void updateInputs(ShooterInputs inputs){
        updateSimulation();
        inputs.isMotorConnected = true; 
        inputs.acc = Units.radiansToRotations(Sim.getAngularAccelerationRadPerSecSq());
        inputs.velocity = Sim.getAngularVelocityRPM();
    }
    @Override
    public void setSpeed(double speed){
        m_shooter.setControl(motionMagicVelocityVoltage.withVelocity(speed));
    }
    @Override
    public void setVolts(double volts){
        m_shooter.setVoltage(volts);
    }
    @Override
    public void updatePidf(){
        m_shooter.getConfigurator().apply(new Slot0Configs()
        .withKP(ShooterConstants.MotionMagicConstants.KP.get())
        .withKI(ShooterConstants.MotionMagicConstants.KI.get())
        .withKD(ShooterConstants.MotionMagicConstants.KD.get())
        .withKV(ShooterConstants.MotionMagicConstants.KV.get())
        .withKA(ShooterConstants.MotionMagicConstants.KA.get())
        .withKS(ShooterConstants.MotionMagicConstants.KS.get())
        .withKG(ShooterConstants.MotionMagicConstants.KG.get()));
    }
}
