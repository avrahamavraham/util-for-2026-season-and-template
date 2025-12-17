// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/** Add your docs here. */
public class GripperIOSim implements GripperIO {
    private FlywheelSim Sim;
    private TalonFX m_Gripper = new TalonFX(GripperConstants.m_MotorId, GripperConstants.m_CanBusName);
    private MotionMagicVelocityVoltage VelocityVoltage = new MotionMagicVelocityVoltage(0);

    public GripperIOSim() {
        Sim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(GripperConstants.dcMotor, 0.001, GripperConstants.gearRatio),
                GripperConstants.dcMotor);
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        FeedbackConfigs feedbackConfigsspin = talonFXConfiguration.Feedback;
        feedbackConfigsspin.SensorToMechanismRatio = GripperConstants.POSITION_CONVERSION_FACTOR;
        MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
        motorOutputConfigs.NeutralMode = GripperConstants.NeutralMode;
        MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = GripperConstants.MotionMagicConstants.MOTION_MAGIC_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = GripperConstants.MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = GripperConstants.MotionMagicConstants.MOTION_MAGIC_JERK;
        motionMagicConfigs.MotionMagicExpo_kA = GripperConstants.MotionMagicConstants.Expo_KV.get();
        motionMagicConfigs.MotionMagicExpo_kV = GripperConstants.MotionMagicConstants.Expo_KV.get();

        talonFXConfiguration.Slot0 = GripperConstants.MotionMagicConstants.slot0Configs;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = m_Gripper.getConfigurator().apply(talonFXConfiguration);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not configure device. Error: " + status.toString());
        }
    }

    @Override
    public void updateInputs(GripperInputs inputs) {
        Sim.setInputVoltage(m_Gripper.getSimState().getMotorVoltage());
        Sim.update(0.02);
        inputs.velocity = Units.radiansToDegrees(Sim.getAngularVelocityRPM());
        inputs.acceleration = Units.radiansToDegrees(Sim.getAngularVelocityRadPerSec());
        inputs.isMotorConnect = true;
        inputs.voltage = Sim.getInputVoltage();
        inputs.haveGamePiece = true;
    }

    @Override
    public void setVolts(double volts) {
        m_Gripper.setVoltage(volts);
    }

    @Override
    public void setVelocity(double velocity) {
        m_Gripper.setControl(VelocityVoltage.withVelocity(velocity));
    }

    @Override
    public void hold() {
        setVelocity(0); // put hold velocity of the game piece
    }
}
