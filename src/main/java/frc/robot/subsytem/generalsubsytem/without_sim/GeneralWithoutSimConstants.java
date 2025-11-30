// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.generalsubsytem.without_sim;

import com.ctre.phoenix6.signals.GravityTypeValue;

/** Add your docs here. */
public class GeneralWithoutSimConstants {
    /** the id of the motor subsystem */
    public static final int m_MotorId = 0;
    /** the can bus name defualt is rio */
    public static final String m_CanBusName = "rio";
    public final class MotionMagicConstants {
        public static final double MOTION_MAGIC_VELOCITY = 0; //TODO : this must be tune to a mechanism
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_JERK = 0;
        public static final double MotionMagicExpo_kA = 0;
        public static final double MotionMagicExpo_kV = 0;

        public static final double MOTOR_KS = 0; // TODO INITILIZE THESE VALUES
        public static final double MOTOR_KA = 0;
        public static final double MOTOR_KV = 0;
        public static final double MOTOR_KG = 0;
        public static final double MOTOR_KP = 0;
        public static final double MOTOR_KI = 0;
        public static final double MOTOR_KD = 0;
        /** what the gravity type ?  {@link GravityTypeValue#Arm_Cosine} for Arm (can fall to either side)
         *  {@link GravityTypeValue#Elevator_Static} for an Elevator (can fall only down)*/
        public static final GravityTypeValue GravityType = GravityTypeValue.Arm_Cosine;
        }
}
