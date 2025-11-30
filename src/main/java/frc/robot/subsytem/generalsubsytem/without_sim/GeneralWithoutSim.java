// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.generalsubsytem.without_sim;

import static edu.wpi.first.units.Units.Degree;

import java.lang.Thread.State;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsytem.generalsubsytem.withsim.GeneralWithSimConstants;
import frc.robot.subsytem.generalsubsytem.withsim.Generalstate;

public class GeneralWithoutSim extends SubsystemBase {
  TalonFX subsystemMotor = new TalonFX(0);
  MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
  Generalstate state = Generalstate.KeepItIn;
  /** Creates a new GeneralWithoutSim. */
  public GeneralWithoutSim() {
            TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
            FeedbackConfigs feedbackConfigsspin = talonFXConfiguration.Feedback;
            feedbackConfigsspin.SensorToMechanismRatio = GeneralWithSimConstants.isArm ? 
            GeneralWithSimConstants.rotaion_Sim_Constants.POSITION_CONVERSION_FACTOR :
             GeneralWithSimConstants.Elevator_Sim_Constants.POSITION_CONVERSION_FACTOR;
            MotorOutputConfigs motorOutputConfigs = talonFXConfiguration.MotorOutput;
            motorOutputConfigs.NeutralMode = GeneralWithSimConstants.NeutralMode;
            MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
            motionMagicConfigs.MotionMagicCruiseVelocity =
            GeneralWithSimConstants.MotionMagicConstants.MOTION_MAGIC_VELOCITY;
            motionMagicConfigs.MotionMagicAcceleration =
            GeneralWithSimConstants.MotionMagicConstants.MOTION_MAGIC_ACCELERATION;
            motionMagicConfigs.MotionMagicJerk =
            GeneralWithSimConstants.MotionMagicConstants.MOTION_MAGIC_JERK;
            motionMagicConfigs.MotionMagicExpo_kA = 
            GeneralWithSimConstants.MotionMagicConstants.MotionMagicExpo_kA;
            motionMagicConfigs.MotionMagicExpo_kV = 
            GeneralWithSimConstants.MotionMagicConstants.MotionMagicExpo_kV;

        Slot0Configs slot0 = talonFXConfiguration.Slot0;
            slot0.kS = GeneralWithSimConstants.MotionMagicConstants.MOTOR_KS;
            slot0.kG = GeneralWithSimConstants.MotionMagicConstants.MOTOR_KG;
            slot0.kV = GeneralWithSimConstants.MotionMagicConstants.MOTOR_KV;
            slot0.kA = GeneralWithSimConstants.MotionMagicConstants.MOTOR_KA;
            slot0.kP = GeneralWithSimConstants.MotionMagicConstants.MOTOR_KP;
            slot0.kI = GeneralWithSimConstants.MotionMagicConstants.MOTOR_KI;
            slot0.kD = GeneralWithSimConstants.MotionMagicConstants.MOTOR_KD;
            slot0.GravityType = GeneralWithSimConstants.MotionMagicConstants.GravityType;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = subsystemMotor.getConfigurator().apply(talonFXConfiguration);
            if (status.isOK()) break;
        }
            if (!status.isOK()) {
        System.out.println("Could not configure device. Error: " + status.toString());
        }
        subsystemMotor.setControl(motionMagicExpoVoltage.withPosition(GeneralWithSimConstants.rotaion_Sim_Constants.startingAngle)
        .withSlot(0));
        this.setDefaultCommand(this.defualtCommand());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setDegree(double degree){
    motionMagicExpoVoltage.withPosition(degree);
  }
  public Command setDegreeCommand(double degree){
    return Commands.runOnce(()-> setDegree(degree));
  }
  public void setMotorPos(double degree){
    subsystemMotor.setPosition(degree);
  }
  public Command setMotorPosCommand(double degree){
    return Commands.runOnce(()-> setMotorPos(degree));
  }
  public Command defualtCommand(){
    return setDegreeCommand(state.getTarget().in(Degree));
  }
}
