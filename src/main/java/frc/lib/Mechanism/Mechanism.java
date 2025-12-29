// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.Mechanism;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.xml.crypto.Data;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import frc.robot.Robot;
import frc.robot.subsystems.Turret.Turret;

/** Add your docs here. */
public class Mechanism {
    /** use each for what is the mechanism */
    public enum MechanismType {
        /** for Elevator and mechanism that go up and dwon in the smae direction */
        Elevator,
        /** for Arm and mechanism that go in a rotational way */
        Arm,
        /** for shooter and things that need to spin fast and not go to a specific place */
        Flywheel,
        /** for evrything else */
        dcMotor;
    }
    private class MechanismData{
        private DoubleSupplier position;
        private DoubleSupplier velocity;
        private DoubleSupplier accelration;
        private DoubleSupplier voltage;
        private BooleanSupplier isConnected;

        public MechanismData (
                DoubleSupplier position, DoubleSupplier velocity, DoubleSupplier accelration,
                DoubleSupplier voltage,BooleanSupplier isConnected) {
            this.position = position;
            this.velocity = velocity;
            this.accelration = accelration;
            this.voltage = voltage;
            this.isConnected = isConnected;
        }
        public boolean isConncted(){
            return isConnected.getAsBoolean();
        }
        public double getPosition() {
            return position.getAsDouble();
        }

        public double getVelocity() {
            return velocity.getAsDouble();
        }

        public double getAccelration() {
            return accelration.getAsDouble();
        }

        public double getVoltage() {
            return voltage.getAsDouble();
        }
    }

    private TalonFX Motor;
    private SimulationWrapper simulation;
    private double positionFactorForSimulation;
    private MechanismData data;
    public Mechanism(TalonFX Motor, SimulationConfig config) {
        this.Motor = Motor;
        if (config != null && Robot.isSimulation()) {
            simulation = new SimulationWrapper(config);
        }
        positionFactorForSimulation = config.mechanismType == MechanismType.Elevator
                ? config.JkMeterSquaerdOrDrum * 2 * Math.PI * config.gearRatio
                : config.gearRatio;
        if (Robot.isReal()) {
            data = new MechanismData(Motor.getPosition()::getValueAsDouble, 
            Motor.getVelocity()::getValueAsDouble, 
            Motor.getAcceleration()::getValueAsDouble, 
            Motor.getMotorVoltage()::getValueAsDouble, 
            Motor::isConnected);    
        } else{
            data = new MechanismData(
                simulation::getPosition, 
                simulation::getVelocity, 
                simulation::getAccelration, 
                simulation::getVoltage, 
                ()-> true);
        }
        
    }

    public void periodic() {
        
    }
    public void updateVisual(){

    }

    public void simulationPeriodic() {
        simulation.setVoltage(Motor.getSimState().getMotorVoltage());
        simulation.update(0.02);
        Motor.getSimState().setRawRotorPosition(simulation.getPosition() * positionFactorForSimulation);
        Motor.getSimState().setRotorVelocity(simulation.getVelocity());
        Motor.getSimState().setRotorAcceleration(simulation.getAccelration());
    }
}
