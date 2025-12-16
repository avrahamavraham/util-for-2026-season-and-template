package frc.robot;

import frc.robot.subsytem.ArmWithSim.ArmState;
import frc.robot.subsytem.ElevatorWithSim.ElevatorState;
import frc.robot.subsytem.Gripper.GripperState;

public class SubsystemState {
    public static ElevatorState elevatorState = ElevatorState.colse;
    public static ArmState armState = ArmState.up;
    public static GripperState gripperState = GripperState.STOP;
    public static void resetState(){
        elevatorState = ElevatorState.colse;
        armState = ArmState.up;
        gripperState = GripperState.STOP;
    }
}
