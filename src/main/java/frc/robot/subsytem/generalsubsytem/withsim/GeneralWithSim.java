// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.Generalsubsytem.withsim;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GeneralWithSim extends SubsystemBase {
  /** the state the motor follow */
  @AutoLogOutput
  Generalstate state = Generalstate.KeepItIn;
  GeneralIO io;
  GeneralInputsAutoLogged inputs;
  /** Creates a new GeneralSubsystem. */
  public GeneralWithSim(GeneralIO io) {
    this.io = io;
    io.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GeneralWithSim", inputs);
  }
  @Override
  public void simulationPeriodic() {
    io.updateSim();
  }
}
