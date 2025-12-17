// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ArmPitch;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemState;
import frc.robot.subsystems.ElevatorWithSim.Elevator;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class ArmPitch extends SubsystemBase {
    private ArmPitchIO io;
    private ArmPithcInputsAutoLogged inputs = new ArmPithcInputsAutoLogged();
    private LoggedMechanismLigament2d ligament2d =
            new LoggedMechanismLigament2d("ArmPitch", 5, ArmPitchConstants.startingAngle);
    /** Creates a new ArmWithSim. */
    public ArmPitch(ArmPitchIO io) {
        this.io = io;
        io.updateInputs(inputs);
        Elevator.ligament2dClose.append(ligament2d);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        Logger.processInputs("ArmPitch", inputs);
    }

    public void setState(ArmPitchState state) {
        SubsystemState.armState = state;
    }
    /** set to what degree it shold go */
    public void setDegree(double degree) {
        io.SetAngle(degree);
    }

    public void setVolts(double volts) {
        io.setVolts(volts);
    }
    /** set the motor to that pos */
    public void setPos(double pos) {
        io.setMotorPos(pos);
    }

    public void updateViusal() {
        ligament2d.setAngle(inputs.position);
    }
}
