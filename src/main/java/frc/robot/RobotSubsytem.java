// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsytem.ElevatorWithSim.Elevator;
import frc.robot.subsytem.ElevatorWithSim.ElevatorIOSim;
import frc.robot.subsytem.drive.Drive;
import frc.robot.subsytem.drive.GyroIO;
import frc.robot.subsytem.drive.ModuleIOSim;
import frc.robot.subsytem.drive.TunerConstants;

/** Add your docs here. */
public class RobotSubsytem {
    // GeneralWithSim general = new GeneralWithSim(new GeneralIOSim(SingleJointedArmSim.class.getName()));
    // GeneralWithoutSim general = new GeneralWithoutSim();
    
    public Elevator elevator = new Elevator(new ElevatorIOSim());
    private static SwerveDriveSimulation simulation = new SwerveDriveSimulation(Drive.mapleSimConfig, new Pose2d(2,2,new Rotation2d()));
    public Drive drive = new Drive(new GyroIO() {}, 
    new ModuleIOSim(TunerConstants.FrontLeft),
    new ModuleIOSim(TunerConstants.FrontRight),
    new ModuleIOSim(TunerConstants.BackLeft),
    new ModuleIOSim(TunerConstants.BackRight));
    public RobotSubsytem(){
        
    }
}
