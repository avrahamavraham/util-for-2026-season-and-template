// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytem.elevatorWithSim;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class ElevatorConstants {
    /** the id of the master motor */
    public static final int m_MasterId = 40;
    /** the id for the slave motor */
    public static final int m_Slaveid = 41;
    /** the can bus name defualt is rio */
    public static final String m_CanBusName = "rio";
    /** the switch that is at the butoom of the elevator */
    public static final int m_DIO_Switch = 2;
    /** what mod dose the motor be in when idel*/
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;
    /** position factor that cahnge from the sensor to the acual meters of the  mechanism*/
    public static final double POSITION_CONVERSION_FACTOR = (1/(SimConstants.gearRatio*SimConstants.drumMetersRiauds));
    /** Min and Max Hight in meters*/
    public static final double minHightMeters = 0;
    public static final double maxHightMeters = 3;
    /**the starting hight of the elevator */
    public static final double startingHight = 0;
    public class SimConstants {
    /** the  DC motor that is used*/
    public static final DCMotor dcMotor = DCMotor.getKrakenX60(2);
    /** gear ratio of the mechanism */
    public static final double gearRatio = 34.73;
    /** the mass of the mechanism */
    public static final double MassKg = 9.776;
    /**the radius of the drum in the elevator in centimter */
    public static final double drumMetersRiauds = 0.054;
    }
    public class MotionMagicConstants {
        public static final LoggedNetworkNumber KP = new LoggedNetworkNumber("Elevator/TalonFX/KP",0 );
        public static final LoggedNetworkNumber KI = new LoggedNetworkNumber("Elevator/TalonFX/KI",0 );
        public static final LoggedNetworkNumber KD = new LoggedNetworkNumber("Elevator/TalonFX/KD",0 );
        public static final LoggedNetworkNumber KV = new LoggedNetworkNumber("Elevator/TalonFX/KV",0 );
        public static final LoggedNetworkNumber KA = new LoggedNetworkNumber("Elevator/TalonFX/KA",0 );
        public static final LoggedNetworkNumber KS = new LoggedNetworkNumber("Elevator/TalonFX/KS",0 );
        public static final LoggedNetworkNumber KG = new LoggedNetworkNumber("Elevator/TalonFX/KG",0 );
        public static final LoggedNetworkNumber Expo_KV = new LoggedNetworkNumber("Elevator/TalonFX/EXPO_KV",0);
        public static final LoggedNetworkNumber Expo_KA = new LoggedNetworkNumber("Elevator/TalonFX/EXPO_KA",0);
        public static final Slot0Configs slot0Configs = new Slot0Configs()
        .withKP(KP.get())
        .withKI(KI.get())
        .withKD(KD.get())
        .withKV(KV.get())
        .withKA(KA.get())
        .withKS(KS.get())
        .withKG(KG.get())
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
        public static final double MOTION_MAGIC_ACCELERATION = 0;
        public static final double MOTION_MAGIC_VELOCITY = 0;
        public static final double MOTION_MAGIC_JERK = 0;
    }
}
