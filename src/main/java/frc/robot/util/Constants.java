package frc.robot.util;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants
{
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }


    public final class con_Controllers
    {
        public static final double driver_leftStick_deadband = 0.025;
        //rumble??
    }

    

}
