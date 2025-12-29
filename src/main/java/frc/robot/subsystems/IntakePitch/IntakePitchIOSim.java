// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePitch;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class IntakePitchIOSim implements IntakePitchIO {
    private TalonFX m_intake = new TalonFX(IntakePitchConstants.m_MotorId, IntakePitchConstants.m_CanBusName);
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private SingleJointedArmSim Sim;

    public IntakePitchIOSim() {
        Sim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        IntakePitchConstants.dcMotor,
                        IntakePitchConstants.JKgMeterSqured,
                        IntakePitchConstants.gearRatio),
                IntakePitchConstants.dcMotor,
                IntakePitchConstants.gearRatio,
                IntakePitchConstants.lengthMeters,
                IntakePitchConstants.minAngleRads,
                IntakePitchConstants.maxAngleRads,
                true,
                IntakePitchConstants.startingPosition);
    }
}
