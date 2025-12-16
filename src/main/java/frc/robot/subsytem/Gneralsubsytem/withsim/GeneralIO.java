package frc.robot.subsytem.Gneralsubsytem.withsim;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface GeneralIO {
        public static class GeneralInputs implements LoggableInputs{
        public Angle rotaion;
        public AngularVelocity speed;
        public boolean MotorConncted = false;
        public boolean isClosed = true;
        @Override
        public void toLog(LogTable table) {
            table.put("Rotaion", rotaion);
            table.put("Speed", speed);
            table.put("MotorConncted", MotorConncted);
            table.put("IsClosed", isClosed);
        }

        @Override
        public void fromLog(LogTable table) {
            rotaion = table.get("Rotaion", rotaion);
            speed = table.get("Speed", speed);
            MotorConncted = table.get("MotorConncted", MotorConncted);
            isClosed = table.get("IsClosed", isClosed);
        }

        public GeneralInputs clone() {
            GeneralInputs copy = new GeneralInputs();
            copy.rotaion = this.rotaion;
            copy.speed = this.speed;
            copy.MotorConncted = this.MotorConncted;
            copy.isClosed = this.isClosed;
            return copy;
        }
    }
    /**update inputs from the sim/real motors
     * @param inputs the inputs to update call at the {@link SubsystemBase#periodic()}
    */
    public default void updateInputs(GeneralInputs inputs){}
    /** set rotaion for the motor sim 
     * @param rotaion the setpoint that you want to go to
    */
    public default void setPosition(Angle angle){}
    /** set the speed of the motor 
     * @param speed the speed the motor shold speen at voltz that given
     * @deprecated use setRotaion insted for beter control {@link GeneralIO#setRotaion(DoubleSupplier)}*/
    @Deprecated
    public default void setVoltz(double Vlotz){}
    /** set motor roataion 
     * @param rotaion the rotaion to set the motor to
    */
    public default void setMotorAngle(Angle rotaion){}
    /**update the sim of the subsystem */
    public default void updateSim(){};
    /** update the moudle of the Sim */
    public default void updateVisual(MechanismLigament2d mechanismLigament2d){}
}
