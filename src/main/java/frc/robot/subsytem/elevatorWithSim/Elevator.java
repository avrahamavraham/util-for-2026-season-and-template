// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.ElevatorWithSim;

import static edu.wpi.first.units.Units.Volt;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.RobotContainer;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();
  private SysIdRoutine sysid;
  private ElevatorState state = ElevatorState.colse;
  private LoggedMechanism2d mechanism2d = new LoggedMechanism2d(10, 10,new Color8Bit(Color.kBlue));
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
    LoggedMechanismRoot2d root2dFarLeft = mechanism2d.getRoot("ElevatorFarLeft", 3.7, 0);
    LoggedMechanismRoot2d root2dFarRight = mechanism2d.getRoot("ElevatorFarRight", 6.3, 0);
    LoggedMechanismRoot2d root2dCloseLeft = mechanism2d.getRoot("ElevatorCloseLeft", 6, 0);
    LoggedMechanismRoot2d root2dCloseRight = mechanism2d.getRoot("ElevatorCloseRight", 4, 0);
    root2dCloseLeft.append(ligament2dClose);
    root2dCloseRight.append(ligament2dClose);
    root2dFarRight.append(ligament2dFar);
    root2dFarLeft.append(ligament2dFar);
    Logger.recordOutput("Elevator/mec", mechanism2d);


    this.io = io;
    io.updateInputs(inputs);
    sysid = new SysIdRoutine(
    new Config(null,null,null,
    (state)-> Logger.recordOutput("Elevator/sysid", state)),
     new Mechanism((voltz)-> io.setSpeed(voltz.in(Volt)), null, this));
     this.setDefaultCommand(defualtCommand());
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
    Logger.recordOutput("Elevator/mec", mechanism2d);
  }
  public Command setHeight(double Height){
    return Commands.runOnce(()->{

      io.setHeight(Height);});
  }
  public Command setSpeed(double speed){
    return this.runOnce(()-> io.setSpeed(speed));
  }
  public void defualt(){
    this.setHeight(state.getTarget());
  }
  public Command defualtCommand(){
    return this.runOnce(()-> defualt());
  }
  public void changeState(ElevatorState new_state){
    this.state = new_state;
  }
  public Command changeStateCommand(ElevatorState new_State){
    return Commands.runOnce(()-> changeState(new_State));
  }
  public Command sysidDynamic(SysIdRoutine.Direction direction){
    return Commands.runOnce(()-> io.setSpeed(0)).andThen(sysid.dynamic(direction).andThen(Commands.runOnce(()-> io.setSpeed(0))));
  }
  public Command sysidQuasistatic(SysIdRoutine.Direction direction){
    return Commands.runOnce(()-> io.setSpeed(0)).andThen(sysid.quasistatic(direction).andThen(Commands.runOnce(()-> io.setSpeed(0))));
  }
  public Command PIDFapplieCommand(){
    return this.runOnce(()-> io.appliePIDF());
  }
  private double FF_START_DELAY =0;
  private double FF_RAMP_RATE =0.1;
  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  io.setSpeed(0.0);
                },
                this)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  io.setSpeed(voltage);
                  velocitySamples.add(inputs.Height * ElevatorConstants.POSITION_CONVERSION_FACTOR);
                  voltageSamples.add(voltage);
                },
                this)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumVelocity = 0.0;
                  double sumVoltage = 0.0;
                  double sumVelocityVoltage = 0.0;
                  double sumVelocity2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumVelocity += velocitySamples.get(i);
                    sumVoltage += voltageSamples.get(i);
                    sumVelocityVoltage += velocitySamples.get(i) * voltageSamples.get(i);
                    sumVelocity2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumVoltage * sumVelocity2 - sumVelocity * sumVelocityVoltage) / (n * sumVelocity2 - sumVelocity * sumVelocity);
                  double kV = (n * sumVelocityVoltage - sumVelocity * sumVoltage) / (n * sumVelocity2 - sumVelocity * sumVelocity);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Elevaotr FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

}
