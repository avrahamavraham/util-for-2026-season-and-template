// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Gripper;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.system.plant.DCMotor;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class GripperConstants {
    /** the DC motor that is used */
    public static final DCMotor dcMotor = DCMotor.getKrakenX60(1);
    /** the id of the motor subsystem */
    public static final int m_MotorId = 0;
    /** the can bus name defualt is rio */
    public static final String m_CanBusName = "rio";
    /** id of the DOI */
    public static final int m_DioID = 1;
    /** what mod dose the motor be in when idel */
    public static final NeutralModeValue NeutralMode = NeutralModeValue.Brake;
    /** gear ratio of the mechanism */
    public static final double gearRatio = 0;
    /** the length of the mechanism */
    public static final double lengthMeters = 0;
    /** the MOI for the mechanism get is from CAD (אתה לוקח את זה מסרטוט) */
    public static final double JKgMeterSqured = 0;
    /** position factor that cahnge from the sensor to the acual degree of the mechanism */
    public static final double POSITION_CONVERSION_FACTOR = (1 / (gearRatio * 360));

    public final class MotionMagicConstants {
        public static final LoggedNetworkNumber KP = new LoggedNetworkNumber("Tuning/Gripper/KP", 0);
        public static final LoggedNetworkNumber KI = new LoggedNetworkNumber("Tuning/Gripper/KI", 0);
        public static final LoggedNetworkNumber KD = new LoggedNetworkNumber("Tuning/Gripper/KD", 0);
        public static final LoggedNetworkNumber KV = new LoggedNetworkNumber("Tuning/Gripper/KV", 0);
        public static final LoggedNetworkNumber KA = new LoggedNetworkNumber("Tuning/Gripper/KA", 0);
        public static final LoggedNetworkNumber KS = new LoggedNetworkNumber("Tuning/Gripper/KS", 0);
        public static final LoggedNetworkNumber KG = new LoggedNetworkNumber("Tuning/Gripper/KG", 0);
        public static final LoggedNetworkNumber Expo_KV = new LoggedNetworkNumber("Tuning/Gripper/EXPO_KV", 0);
        public static final LoggedNetworkNumber Expo_KA = new LoggedNetworkNumber("Tuning/Gripper/EXPO_KA", 0);
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
