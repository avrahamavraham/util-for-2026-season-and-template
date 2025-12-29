// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmPitch;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmPitchIOTalonFx implements ArmPitchIO {
    private TalonFX m_Arm;
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(ArmPitchConstants.startingAngle);

    public ArmPitchIOTalonFx() {
        m_Arm = new TalonFX(ArmPitchConstants.m_MotorId, ArmPitchConstants.m_CanBusName);
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        FeedbackConfigs feedbackConfigsspin = talonFXConfiguration.Feedback;
        feedbackConfigsspin.SensorToMechanismRatio = ArmPitchConstants.POSITION_CONVERSION_FACTOR;
        MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = ArmPitchConstants.NeutralMode;
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmPitchConstants.MotionMagicConstants.MOTION_MAGIC_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = ArmPitchConstants.MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = ArmPitchConstants.MotionMagicConstants.MOTION_MAGIC_JERK;

        talonFXConfiguration.Slot0 = ArmPitchConstants.MotionMagicConstants.slot0Configs;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_Arm.getConfigurator().apply(talonFXConfiguration);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    @Override
    public void updateInputs(ArmPithcInputs inputs) {
        inputs.isMotorConnect = m_Arm.isConnected();
        inputs.position = m_Arm.getPosition().getValueAsDouble();
        inputs.velocity = m_Arm.getVelocity().getValueAsDouble();
        inputs.voltage = m_Arm.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void SetAngle(double degree) {
        degree = Units.degreesToRadians(degree);
        m_Arm.setControl(motionMagicVoltage.withPosition(degree));
    }

    @Override
    public void setVolts(double volts) {
        m_Arm.setVoltage(volts);
    }

    @Override
    public void appliePIDF() {
        Slot0Configs config = new Slot0Configs()
                .withKP(ArmPitchConstants.MotionMagicConstants.KP.get())
                .withKI(ArmPitchConstants.MotionMagicConstants.KI.get())
                .withKD(ArmPitchConstants.MotionMagicConstants.KD.get())
                .withKV(ArmPitchConstants.MotionMagicConstants.KV.get())
                .withKA(ArmPitchConstants.MotionMagicConstants.KA.get())
                .withKS(ArmPitchConstants.MotionMagicConstants.KS.get())
                .withKG(ArmPitchConstants.MotionMagicConstants.KG.get())
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_Arm.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    @Override
    public void setMotorPos(double pos) {
        m_Arm.setPosition(pos);
    }
}
