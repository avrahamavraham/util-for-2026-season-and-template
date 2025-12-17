// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.SubsystemState;
import frc.robot.subsystems.Gripper.GripperIO.GripperInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Gripper extends SubsystemBase {
    private GripperIO io;
    private GripperInputs inputs;
    /** double use to clac the pos of the gripper for visual */
    private double lastAngle = 90;

    private LoggedMechanismLigament2d verticalLigament2d =
            new LoggedMechanismLigament2d("gripper vertical", 0.4, lastAngle);
    private LoggedMechanismLigament2d horizontalLigament2d =
            new LoggedMechanismLigament2d("gripper horizontal", 0.4, lastAngle - 90);
    private LoggedMechanismRoot2d gripperRoot = RobotContainer.Mechanism2d.getRoot("gripper", 0, 0);

    /** Creates a new Gripper. */
    public Gripper(GripperIO io) {
        this.io = io;
        io.updateInputs(inputs);
        gripperRoot.append(verticalLigament2d);
        gripperRoot.append(horizontalLigament2d);
    }

    @Override
    public void periodic() {
        Logger.processInputs("Gripper", inputs);
        updateVisual(5, 5);
    }

    public void addGamePieceToGripper() {
        inputs.haveGamePiece = true;
    }

    public void removeGamePieceToGripper() {
        inputs.haveGamePiece = false;
    }

    public Command setVelocity(double velocity) {
        return Commands.runOnce(() -> io.setVelocity(velocity));
    }

    public void setState(GripperState state) {
        SubsystemState.gripperState = state;
    }

    public Command setStateCommand(GripperState state) {
        return Commands.runOnce(() -> setState(state));
    }

    public boolean haveGamePiece() {
        return inputs.haveGamePiece;
    }
    /**
     * update the visual of the gripper
     *
     * @param x where the gripper should be in the x dimension
     * @param y where the gripper should be in the y dimension
     */
    public void updateVisual(double x, double y) {
        lastAngle = lastAngle + inputs.velocity;
        gripperRoot.setPosition(x, y);
        verticalLigament2d.setAngle(lastAngle);
        horizontalLigament2d.setAngle(lastAngle - 90);
    }
}
