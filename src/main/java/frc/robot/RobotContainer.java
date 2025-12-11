// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsytem.ElevatorWithSim.ElevatorState;

public class RobotContainer {
  public static LoggedMechanism2d Mechanism2d = new LoggedMechanism2d(10,10,new Color8Bit(Color.kBlack));
  private Subsytem subsytem;
  public RobotContainer() {
    subsytem = new Subsytem();
    configureBindings();
  }

  private void configureBindings() {
    Controler.testController.a().onTrue(subsytem.elevator.PIDFapplieCommand());
    Controler.testController.b().onTrue(subsytem.elevator.changeStateCommand(ElevatorState.level1));
    Controler.testController.x().onTrue(subsytem.elevator.changeStateCommand(ElevatorState.colse));
    Controler.testController.y().onTrue(subsytem.elevator.feedforwardCharacterization());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
