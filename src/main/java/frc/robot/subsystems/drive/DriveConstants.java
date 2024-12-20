package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

public class DriveConstants
{
    public static final NavXUpdateRate odometryFrequency = NavXUpdateRate.k100Hz;
    public static final double trackWidth = edu.wpi.first.math.util.Units.inchesToMeters(26.5);
    public static final double wheelBase = edu.wpi.first.math.util.Units.inchesToMeters(26.5);
    public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
    public static final Translation2d[] moduleTranslations =
        new Translation2d[] {
          new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
          new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };
  
    // Zeroed rotation values for each module, see setup instructions
    public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
    public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);
  
    // Device CAN IDs
    // public static final int pigeonCanId = 9;
  
    // // Drive motor configuration
    //Drive Config
    public static final Slot0Configs driveGains = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);
    public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    public static final Current slipCurrent = edu.wpi.first.units.Units.Amps.of(120);
    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    public static final LinearVelocity speedAt12Volts = Units.MetersPerSecond.of(4.55);
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double maxSpeedMetersPerSec = 4.8;
    public static final double coupleRatio = 3.5;
    public static final double driveGearRatio = 4.59;
    public static final Distance wheelRadius = Units.Inches.of(1.5);
    public static final int driveMotorCurrentLimit = 50;
  
    // Drive encoder configuration
    public static final double driveEncoderPositionFactor =
        2 * Math.PI / driveGearRatio; // Rotor Rotations -> Wheel Radians
    public static final double driveEncoderVelocityFactor =
        (2 * Math.PI) / 60.0 / driveGearRatio; // Rotor RPM -> Wheel Rad/Sec
  

    // Turn motor configuration
    //Steer Config
    public static final boolean turnInverted = false;
    public static final int turnMotorCurrentLimit = 20;
    public static final double turnMotorReduction = 9424.0 / 203.0;
    public static final DCMotor turnGearbox = DCMotor.getNEO(1);
    public static final SparkBaseConfig steerGains = new SparkMaxConfig().apply(new ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
    public static final double steerGearRatio = 13.3714;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

    
    // Turn encoder configuration
    public static final boolean turnEncoderInverted = true;
    public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
    public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec


    //CanCoder Config
    public static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
    public static final CANBus kCANBus = new CANBus("rio", "./logs/example.hoot");

    // Turn PID configuration
    public static final double turnKp = 2.0;
    public static final double turnKd = 0.0;


    // These are only used for simulation
    public static final double steerInertia = 0.004;
    public static final double driveInertia = 0.025;
    public static final double turnSimP = 0.5;
    public static final double turnSimD = 0.5;
    public static final double driveSimP = 0.5;
    public static final double driveSimD = 0.0;
    // Simulated voltage necessary to overcome friction
    public static final Voltage steerFrictionVoltage = Units.Volts.of(0.25);
    public static final Voltage driveFrictionVoltage = Units.Volts.of(0.25);


    //Hello
    // Front Left
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 1;
    public static final int kFrontLeftEncoderId = 1;
    public static final Rotation2d kFrontLeftZeroRotation = new Rotation2d(0.0);
    public static final Angle kFrontLeftEncoderOffset = Units.Rotations.of(0);
    public static final boolean kFrontLeftSteerMotorInverted = true;
    public static final boolean kFrontLeftCANcoderInverted = false;
    public static final boolean kFrontLeftDriveInverted = false;
    
    public static final Distance kFrontLeftXPos = Units.Inches.of(17);//forward positive
    public static final Distance kFrontLeftYPos = Units.Inches.of(17);//left positive
    
    // Front Right
    public static final int kFrontRightDriveMotorId = 2;
    public static final int kFrontRightSteerMotorId = 2;
    public static final int kFrontRightEncoderId = 2;
    public static final Rotation2d kFrontRightZeroRotation = new Rotation2d(0.0);
    public static final Angle kFrontRightEncoderOffset = Units.Rotations.of(0);
    public static final boolean kFrontRightSteerMotorInverted = true;
    public static final boolean kFrontRightCANcoderInverted = false;
    public static final boolean kFrontRightDriveInverted = false;
    
    public static final Distance kFrontRightXPos = Units.Inches.of(17);
    public static final Distance kFrontRightYPos = Units.Inches.of(-17);
    
    // Back Left
    public static final int kBackLeftDriveMotorId = 3;
    public static final int kBackLeftSteerMotorId = 3;
    public static final int kBackLeftEncoderId = 3;
    public static final Rotation2d kBackLeftZeroRotation = new Rotation2d(0.0);
    public static final Angle kBackLeftEncoderOffset = Units.Rotations.of(0);
    public static final boolean kBackLeftSteerMotorInverted = true;
    public static final boolean kBackLeftCANcoderInverted = false;
    public static final boolean kBackLeftDriveInverted = false;

    public static final Distance kBackLeftXPos = Units.Inches.of(-17);
    public static final Distance kBackLeftYPos = Units.Inches.of(17);
    
    // Back Right
    public static final int kBackRightDriveMotorId = 4;
    public static final int kBackRightSteerMotorId = 4;
    public static final int kBackRightEncoderId = 4;
    public static final Rotation2d kBackRightZeroRotation = new Rotation2d(0.0);
    public static final Angle kBackRightEncoderOffset = Units.Rotations.of(0);
    public static final boolean kBackRightSteerMotorInverted = true;
    public static final boolean kBackRightCANcoderInverted = false;
    public static final boolean kBackRightDriveInverted = false;

    public static final Distance kBackRightXPos = Units.Inches.of(-17);
    public static final Distance kBackRightYPos = Units.Inches.of(-17);


            
    // PathPlanner configuration
    public static final double robotMassKg = 25.8;
    public static final double robotMOI = 6.883;
    public static final double wheelCOF = 1.2;
    public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);
    public static final DCMotor steerGearbox = DCMotor.getNEO(1);
    public static final RobotConfig ppConfig =
    new RobotConfig(
        robotMassKg,
        robotMOI,
        new ModuleConfig(
            wheelRadius.magnitude(),
            maxSpeedMetersPerSec,
            wheelCOF,
            driveGearbox.withReduction(driveGearRatio),
            slipCurrent.magnitude(),
            1),
            trackWidth, wheelBase);
    

        

}
