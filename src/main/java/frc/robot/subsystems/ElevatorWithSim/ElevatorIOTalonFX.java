// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ElevatorWithSim;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
    private TalonFX m_master = new TalonFX(ElevatorConstants.m_MasterId, ElevatorConstants.m_CanBusName);
    private TalonFX m_slave = new TalonFX(ElevatorConstants.m_Slaveid, ElevatorConstants.m_CanBusName);
    private DigitalInput m_closeSwitch = new DigitalInput(ElevatorConstants.m_DIO_Switch);
    private MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(ElevatorConstants.startingHight);

    public ElevatorIOTalonFX() {
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        FeedbackConfigs feedbackConfigsspin = talonFXConfiguration.Feedback;
        feedbackConfigsspin.SensorToMechanismRatio = ElevatorConstants.POSITION_CONVERSION_FACTOR;
        MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = ElevatorConstants.NeutralMode;
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MotionMagicConstants.MOTION_MAGIC_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MotionMagicConstants.MOTION_MAGIC_JERK;
        motionMagicConfigs.MotionMagicExpo_kA = ElevatorConstants.MotionMagicConstants.Expo_KV.get();
        motionMagicConfigs.MotionMagicExpo_kV = ElevatorConstants.MotionMagicConstants.Expo_KV.get();

        talonFXConfiguration.Slot0 = ElevatorConstants.MotionMagicConstants.slot0Configs;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_master.getConfigurator().apply(talonFXConfiguration);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
        m_slave.setControl(new Follower(m_master.getDeviceID(), false));
        m_master.setControl(motionMagicExpoVoltage
                .withPosition(ElevatorConstants.startingHight)
                .withSlot(0));
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.isMotorConnect = m_master.isConnected() && m_slave.isConnected();
        inputs.Height = m_master.getPosition().getValueAsDouble();
        inputs.isClosed = m_closeSwitch.get();
        inputs.speed = m_master.getVelocity().getValueAsDouble();
        inputs.voltage = m_master.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setHeight(Double Height) {
        m_master.setControl(motionMagicExpoVoltage.withPosition(Height));
    }

    @Override
    public void setSpeed(double voltz) {
        m_master.setVoltage(voltz);
    }

    @Override
    public void appliePIDF() {
        m_master.getConfigurator()
                .apply(new Slot0Configs()
                        .withKP(ElevatorConstants.MotionMagicConstants.KP.get())
                        .withKI(ElevatorConstants.MotionMagicConstants.KI.get())
                        .withKD(ElevatorConstants.MotionMagicConstants.KD.get())
                        .withKV(ElevatorConstants.MotionMagicConstants.KV.get())
                        .withKA(ElevatorConstants.MotionMagicConstants.KA.get())
                        .withKS(ElevatorConstants.MotionMagicConstants.KS.get())
                        .withKG(ElevatorConstants.MotionMagicConstants.KG.get()));
        m_master.getConfigurator()
                .apply(new MotionMagicConfigs()
                        .withMotionMagicExpo_kA(ElevatorConstants.MotionMagicConstants.Expo_KA.get())
                        .withMotionMagicExpo_kV(ElevatorConstants.MotionMagicConstants.Expo_KV.get()));
    }
}
