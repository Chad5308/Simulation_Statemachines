package frc.robot.util;

// Contains all ports on our robot

public class RobotMap
{
  public static class mapControllers
  {
    public static final int DRIVER_USB = 0;
    public static final int OPERATOR_USB = 1;
    public static final int TEST_OPERATOR_USB = 5;
  }

  public static class mapDrivetrain
  {
    public static final String CAN_BUS_NAME = "Swerve";
    public static final int NAVX_CAN = 0;

    // Module 0
    public static final int FRONT_LEFT_DRIVE_CAN = 0;
    public static final int FRONT_LEFT_STEER_CAN = 1;
    public static final int FRONT_LEFT_ABSOLUTE_ENCODER_CAN = 0;

    // Module 1
    public static final int FRONT_RIGHT_DRIVE_CAN = 2;
    public static final int FRONT_RIGHT_STEER_CAN = 3;
    public static final int FRONT_RIGHT_ABSOLUTE_ENCODER_CAN = 1;

    // Module 2
    public static final int BACK_LEFT_DRIVE_CAN = 4;
    public static final int BACK_LEFT_STEER_CAN = 5;
    public static final int BACK_LEFT_ABSOLUTE_ENCODER_CAN = 2;

    // Module 3
    public static final int BACK_RIGHT_DRIVE_CAN = 6;
    public static final int BACK_RIGHT_STEER_CAN = 7;
    public static final int BACK_RIGHT_ABSOLUTE_ENCODER_CAN = 3;
  }

//   public static class mapLEDs {
//     public static final int CANDLE_CAN = 60;
//   }
}
