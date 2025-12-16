// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.button;

import frc.robot.Controler;
import frc.robot.RobotSubsytem;
import frc.robot.Commands.DriveCommands;

/** Add your docs here. */
public class DefualtButton {
    public static void loadButton(RobotSubsytem subsytem,Controler controler){
        swerveControl(subsytem,controler);
            }
        
    private static void swerveControl(RobotSubsytem subsytem, Controler controler) {
        subsytem.drive.setDefaultCommand(DriveCommands.joystickDrive(subsytem.drive, ()-> -controler.swerveController.getLeftY() ,
        ()-> -controler.swerveController.getLeftX(), ()-> -controler.swerveController.getRightX()));
    }
}
