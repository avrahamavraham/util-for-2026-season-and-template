package frc.robot.subsytem.elevatorWithSim;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.lib.util.ITargetDistance;

public enum ElevatorState implements ITargetDistance{
    colse(ElevatorConstants.startingHight),
    level1(ElevatorConstants.startingHight + 1);
    private Distance m_height;
    private ElevatorState(double height){
        m_height = Meters.of(height);
    }
    private ElevatorState(Distance height){
        m_height = height;
    }
    public Distance getTarget(){
        return m_height;
    }
}
