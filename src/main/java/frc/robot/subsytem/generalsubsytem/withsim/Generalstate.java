package frc.robot.subsytem.generalsubsytem.withsim;

import static edu.wpi.first.units.Units.Degree;

import edu.wpi.first.units.measure.Angle;
import frc.lib.util.ITargetAngle;

public enum Generalstate implements ITargetAngle{
  /**
   * this is how a state control shold look like 
   */

   /** start with saying all of your varible and then countinue*/
    KeepItIn(0),
    Collect(-1),
    Eject(1);
    /**have the double that you want to store*/
    private Angle m_angle;
    /** a constractor that get a Angle and put it in the Angle vireble*/
    Generalstate(Angle angle){
      m_angle = angle;
    }
    /**get a double and put it as degrees */
    Generalstate(double degree){
      m_angle = Degree.of(degree);
    }

    /** an override method from ITraget that return the value of the double in the enum */
    @Override
    public Angle getTarget() {
      return m_angle;
    }
  }