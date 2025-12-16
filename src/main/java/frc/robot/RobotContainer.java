// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.button.DefualtButton;

public class RobotContainer {
  public static LoggedMechanism2d Mechanism2d = new LoggedMechanism2d(10,10,new Color8Bit(Color.kBlack));
  private RobotSubsytem subsytem;
  private Controler controler;
  public RobotContainer() {
    controler = new Controler();
    subsytem = new RobotSubsytem();
    configureBindings();
  }

  private void configureBindings() {
    DefualtButton.loadButton(subsytem, controler);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
