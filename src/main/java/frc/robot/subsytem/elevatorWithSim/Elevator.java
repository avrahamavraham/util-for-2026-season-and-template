// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.ElevatorWithSim;

import static edu.wpi.first.units.Units.Volt;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.RobotContainer;
import frc.robot.SubsystemState;
import frc.robot.subsytem.ElevatorWithSim.ElevatorIO.ElevatorInputs;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorInputs inputs = new ElevatorInputs();
  private SysIdRoutine sysid;
  private LoggedMechanismRoot2d m_root2dFarLeft = RobotContainer.Mechanism2d.getRoot("ElevatorFarLeft", 3.7, 0);
  private LoggedMechanismRoot2d m_root2dFarRight = RobotContainer.Mechanism2d.getRoot("ElevatorFarRight", 6.3, 0);
  private LoggedMechanismRoot2d m_root2dCloseLeft = RobotContainer.Mechanism2d.getRoot("ElevatorCloseLeft", 6, 0);
  private LoggedMechanismRoot2d m_root2dCloseRight = RobotContainer.Mechanism2d.getRoot("ElevatorCloseRight", 4, 0);
  private LoggedMechanismLigament2d ligament2dFar = new LoggedMechanismLigament2d("ElevatorFar", 1, 90,7,new Color8Bit(Color.kWhite));
  private LoggedMechanismLigament2d ligament2dClose = new LoggedMechanismLigament2d("ElevatorClose", 1, 90,7,new Color8Bit(Color.kWhite));
  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    m_root2dCloseLeft.append(ligament2dClose);
    m_root2dCloseRight.append(ligament2dClose);
    m_root2dFarRight.append(ligament2dFar);
    m_root2dFarLeft.append(ligament2dFar);


    this.io = io;
    io.updateInputs(inputs);
    sysid = new SysIdRoutine(
    new Config(null,null,null,
    (state)-> Logger.recordOutput("Elevator/sysid", state)),
     new Mechanism((voltz)-> io.setSpeed(voltz.in(Volt)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);  
    Logger.processInputs("Elevator", inputs);
  }
  @Override
  public void simulationPeriodic(){
    ligament2dFar.setLength(Math.min(inputs.Height+1,2.5));
    ligament2dClose.setLength(inputs.Height+1);
  }
  // set the height of the mec from a double
  public void setHeight(double Height){
      io.setHeight(Height);
  }
  //in voltz!!!!!!! not duty cycle!!!!
  public Command setSpeed(double speed){
    return this.runOnce(()-> io.setSpeed(speed));
  }
  public void changeState(ElevatorState new_state){
    SubsystemState.elevatorState = new_state;
  }
  // this is how you move the mec
  public Command changeStateCommand(ElevatorState new_State){
    return Commands.runOnce(()-> changeState(new_State));
  }
  //sysid command
  public Command sysidDynamic(SysIdRoutine.Direction direction){
    return Commands.runOnce(()-> io.setSpeed(0)).andThen(sysid.dynamic(direction).andThen(Commands.runOnce(()-> io.setSpeed(0))));
  }
  public Command sysidQuasistatic(SysIdRoutine.Direction direction){
    return Commands.runOnce(()-> io.setSpeed(0)).andThen(sysid.quasistatic(direction).andThen(Commands.runOnce(()-> io.setSpeed(0))));
  }
  // use only when calibration
  public Command PIDFapplieCommand(){
    return this.runOnce(()-> io.appliePIDF());
  }


}
