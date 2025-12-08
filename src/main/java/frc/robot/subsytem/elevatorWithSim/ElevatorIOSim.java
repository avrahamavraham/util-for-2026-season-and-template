// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.elevatorWithSim;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{
    private ElevatorSim Sim;
    private TalonFX m_master = new TalonFX(ElevatorConstants.m_MasterId,ElevatorConstants.m_CanBusName);
    private MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0).withEnableFOC(true);
    public ElevatorIOSim(){
        Sim = new ElevatorSim(LinearSystemId.createElevatorSystem(ElevatorConstants.SimConstants.dcMotor,
         ElevatorConstants.SimConstants.MassKg, ElevatorConstants.SimConstants.drumMetersRiauds,
          ElevatorConstants.SimConstants.gearRatio),
          ElevatorConstants.SimConstants.dcMotor, ElevatorConstants.minHightMeters, ElevatorConstants.maxHightMeters, true, ElevatorConstants.startingHight);
                        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
            FeedbackConfigs feedbackConfigsspin = talonFXConfiguration.Feedback;
            feedbackConfigsspin.SensorToMechanismRatio =
             ElevatorConstants.POSITION_CONVERSION_FACTOR;
            MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
            motorOutputConfigs.NeutralMode = ElevatorConstants.NeutralMode;
            MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity =
            ElevatorConstants.MotionMagicConstants.MOTION_MAGIC_VELOCITY;
            motionMagicConfigs.MotionMagicAcceleration =
            ElevatorConstants.MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
            motionMagicConfigs.MotionMagicJerk =
            ElevatorConstants.MotionMagicConstants.MOTION_MAGIC_JERK;
            motionMagicConfigs.MotionMagicExpo_kA = 
            ElevatorConstants.MotionMagicConstants.Expo_KV.get();
            motionMagicConfigs.MotionMagicExpo_kV = 
            ElevatorConstants.MotionMagicConstants.Expo_KV.get();

            talonFXConfiguration.Slot0 = ElevatorConstants.MotionMagicConstants.slot0Configs;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_master.getConfigurator().apply(talonFXConfiguration);
            if (status.isOK()) break;
        }
            if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
        }
        m_master.setControl(motionMagicExpoVoltage.withPosition(ElevatorConstants.startingHight)
        .withSlot(0));
    }
    @Override
    public void updateInputs(ElevatorInputs inputs){
        var SimMotor = m_master.getSimState();
        Sim.setInputVoltage(SimMotor.getMotorVoltageMeasure().in(Volts));
        Sim.update(0.02);
        SimMotor.setRawRotorPosition(Sim.getPositionMeters() * ElevatorConstants.POSITION_CONVERSION_FACTOR);
        SimMotor.setRotorVelocity(Sim.getVelocityMetersPerSecond() * ElevatorConstants.POSITION_CONVERSION_FACTOR);
        inputs.isMotorConnect = true;
        inputs.Height = Meters.of(Sim.getPositionMeters());
        inputs.speed = MetersPerSecond.of(Sim.getVelocityMetersPerSecond());
        inputs.voltage = Volts.of(Sim.getInput(0));
        inputs.isClosed = Sim.getPositionMeters() < 0.02;
    }
    @Override
    public void setHeight(Distance Height){
        m_master.setControl(motionMagicExpoVoltage.withPosition(Height.in(Centimeter)).withSlot(0));
    }
    @Override
    public void setSpeed(double Volts){
        m_master.setVoltage(Volts);
    }
    @Override
    public void appliePIDF(){
        m_master.getConfigurator().apply(new Slot0Configs()
        .withKP(ElevatorConstants.MotionMagicConstants.KP.get())
        .withKI(ElevatorConstants.MotionMagicConstants.KI.get())
        .withKD(ElevatorConstants.MotionMagicConstants.KD.get())
        .withKV(ElevatorConstants.MotionMagicConstants.KV.get())
        .withKA(ElevatorConstants.MotionMagicConstants.KA.get())
        .withKS(ElevatorConstants.MotionMagicConstants.KS.get())
        .withKG(ElevatorConstants.MotionMagicConstants.KG.get()));
        m_master.getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicExpo_kA(ElevatorConstants.MotionMagicConstants.Expo_KA.get())
        .withMotionMagicExpo_kV(ElevatorConstants.MotionMagicConstants.Expo_KV.get()));
    }
}
