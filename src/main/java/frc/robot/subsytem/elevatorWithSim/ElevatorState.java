package frc.robot.subsytem.ElevatorWithSim;

import frc.lib.util.ITarget;

public enum ElevatorState implements ITarget{
    colse(ElevatorConstants.startingHight),
    level1(ElevatorConstants.startingHight + 1);
    private double m_height;
    private ElevatorState(double height){
        m_height = height;
    }
    public double getTarget(){
        return m_height;
    }
}
