// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.Gripper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemState;
import frc.robot.subsytem.Gripper.GripperIO.GripperInputs;

public class Gripper extends SubsystemBase {
  private GripperIO io;
  private GripperInputs inputs;
  /** Creates a new Gripper. */
  public Gripper(GripperIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    Logger.processInputs("Gripper", inputs);
  }
  public void addGamePieceToGripper(){
    inputs.haveGamePiece = true;
  }
  public void removeGamePieceToGripper(){
    inputs.haveGamePiece = false;
  }
  public Command setVelocity(double velocity){
    return Commands.runOnce(()-> io.setVelocity(velocity));
  }
  public void setState(GripperState state){
    SubsystemState.gripperState = state;
  }
  public Command setStateCommand(GripperState state){
    return Commands.runOnce(()-> setState(state));
  }
  
}
