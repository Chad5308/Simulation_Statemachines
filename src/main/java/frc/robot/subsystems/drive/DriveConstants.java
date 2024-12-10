package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;

public class DriveConstants
{
        
        public static final double odometryFrequency = 100.0; // Hz
        public static final double trackWidth = edu.wpi.first.math.util.Units.inchesToMeters(34);
        public static final double wheelBase = edu.wpi.first.math.util.Units.inchesToMeters(34);
        public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        public static final SwerveDriveKinematics moduleTranslations = new SwerveDriveKinematics(
            new Translation2d(trackWidth / 2, -wheelBase / 2), //front left
            new Translation2d(trackWidth / 2, wheelBase / 2), //front right
            new Translation2d(-trackWidth / 2, -wheelBase / 2), //back left
            new Translation2d(-trackWidth / 2, wheelBase / 2)); //back right
            
            
        
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        //Drive Config
        private static final Slot0Configs driveGains = new Slot0Configs().withKP(0.1).withKI(0).withKD(0.1).withKS(0.4).withKV(0.124);
        
        private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
        private static final Current kSlipCurrent = edu.wpi.first.units.Units.Amps.of(120);
        private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
        public static final LinearVelocity kSpeedAt12Volts = Units.MetersPerSecond.of(4.55);
        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        public static final double maxSpeedMetersPerSec = 4.8;
        private static final double kCoupleRatio = 3.5;
        private static final double kDriveGearRatio = 4.59;
        private static final Distance kWheelRadius = Units.Inches.of(1.5);
        
        
        //Steer Config
        private static final SparkBaseConfig steerGains = new SparkMaxConfig().apply(new 
        ClosedLoopConfig().pidf(0.0075, 0.0, 0.075, 0.0, ClosedLoopSlot.kSlot0));
        private static final double kSteerGearRatio = 13.3714;
        public static final double turnPIDMinInput = 0; // Radians
        public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
        public static final int turnMotorCurrentLimit = 20;
        public static final double turnMotorReduction = 9424.0 / 203.0;
        public static final DCMotor turnGearbox = DCMotor.getNeo550(1);
        public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
        public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

                

        //CanCoder Config
        private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
        public static final CANBus kCANBus = new CANBus("rio", "./logs/example.hoot");
        private static final boolean kInvertLeftSide = false;
        private static final boolean kInvertRightSide = true;


        // These are only used for simulation
        private static final double kSteerInertia = 0.004;
        private static final double kDriveInertia = 0.025;
        public static final double turnSimP = 0.5;
        public static final double driveSimP = 0.5;
        public static final double driveSimD = 0.0;
        // Simulated voltage necessary to overcome friction
        private static final Voltage kSteerFrictionVoltage = Units.Volts.of(0.25);
        private static final Voltage kDriveFrictionVoltage = Units.Volts.of(0.25);

        
        //Hello
        // Front Left
        private static final int kFrontLeftDriveMotorId = 1;
        public static final int kFrontLeftSteerMotorId = 1;
        private static final int kFrontLeftEncoderId = 1;
        public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
        private static final Angle kFrontLeftEncoderOffset = Units.Rotations.of(0);
        private static final boolean kFrontLeftSteerMotorInverted = true;
        private static final boolean kFrontLeftCANcoderInverted = false;
        
        private static final Distance kFrontLeftXPos = Units.Inches.of(17);//forward positive
        private static final Distance kFrontLeftYPos = Units.Inches.of(17);//left positive
        
        // Front Right
        private static final int kFrontRightDriveMotorId = 2;
        public static final int kFrontRightSteerMotorId = 2;
        private static final int kFrontRightEncoderId = 2;
        public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
        private static final Angle kFrontRightEncoderOffset = Units.Rotations.of(0);
        private static final boolean kFrontRightSteerMotorInverted = true;
        private static final boolean kFrontRightCANcoderInverted = false;
        
        private static final Distance kFrontRightXPos = Units.Inches.of(17);
        private static final Distance kFrontRightYPos = Units.Inches.of(-17);
        
        // Back Left
        private static final int kBackLeftDriveMotorId = 3;
        public static final int kBackLeftSteerMotorId = 3;
        private static final int kBackLeftEncoderId = 3;
        public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
        private static final Angle kBackLeftEncoderOffset = Units.Rotations.of(0);
        private static final boolean kBackLeftSteerMotorInverted = true;
        private static final boolean kBackLeftCANcoderInverted = false;
        
        private static final Distance kBackLeftXPos = Units.Inches.of(-17);
        private static final Distance kBackLeftYPos = Units.Inches.of(17);
        
        // Back Right
        private static final int kBackRightDriveMotorId = 4;
        public static final int kBackRightSteerMotorId = 4;
        private static final int kBackRightEncoderId = 4;
        public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);
        private static final Angle kBackRightEncoderOffset = Units.Rotations.of(0);
        private static final boolean kBackRightSteerMotorInverted = true;
        private static final boolean kBackRightCANcoderInverted = false;
        
        private static final Distance kBackRightXPos = Units.Inches.of(-17);
        private static final Distance kBackRightYPos = Units.Inches.of(-17);




           // PathPlanner configuration
           public static final double robotMassKg = 25.8;
           public static final double robotMOI = 6.883;
           public static final double wheelCOF = 1.2;
           public static final DCMotor driveGearbox = DCMotor.getNEO(1);
           public static final RobotConfig ppConfig =
           new RobotConfig(
               robotMassKg,
               robotMOI,
               new ModuleConfig(
                   kWheelRadius.magnitude(),
                   maxSpeedMetersPerSec,
                   wheelCOF,
                   driveGearbox.withReduction(kDriveGearRatio),
                   kSlipCurrent.magnitude(),
                   1),
                   trackWidth, wheelBase);
           
           

        // public static final SwerveDrivetrainConstants DrivetrainConstants =
        //     new SwerveDrivetrainConstants()
        //         .withCANBusName(kCANBus.getName())
        //         .withPigeon2Id(kPigeonId)
        //         .withPigeon2Configs(pigeonConfigs);

        private static final SwerveModuleConstantsFactory ConstantCreator =
            new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .withCouplingGearRatio(kCoupleRatio)
                .withWheelRadius(kWheelRadius)
                // .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                // .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
                .withSlipCurrent(kSlipCurrent)
                .withSpeedAt12Volts(kSpeedAt12Volts)
                // .withFeedbackSource(kSteerFeedbackType)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                // .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withCANcoderInitialConfigs(cancoderInitialConfigs)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveFrictionVoltage(kDriveFrictionVoltage);
        
        public static final SwerveModuleConstants FrontLeft =
            ConstantCreator.createModuleConstants(
                kFrontLeftSteerMotorId,
                kFrontLeftDriveMotorId,
                kFrontLeftEncoderId,
                kFrontLeftEncoderOffset,
                kFrontLeftXPos,
                kFrontLeftYPos,
                kInvertLeftSide,
                kFrontLeftSteerMotorInverted,
                kFrontLeftCANcoderInverted);
        public static final SwerveModuleConstants FrontRight =
            ConstantCreator.createModuleConstants(
                kFrontRightSteerMotorId,
                kFrontRightDriveMotorId,
                kFrontRightEncoderId,
                kFrontRightEncoderOffset,
                kFrontRightXPos,
                kFrontRightYPos,
                kInvertRightSide,
                kFrontRightSteerMotorInverted,
                kFrontRightCANcoderInverted);
        public static final SwerveModuleConstants BackLeft =
            ConstantCreator.createModuleConstants(
                kBackLeftSteerMotorId,
                kBackLeftDriveMotorId,
                kBackLeftEncoderId,
                kBackLeftEncoderOffset,
                kBackLeftXPos,
                kBackLeftYPos,
                kInvertLeftSide,
                kBackLeftSteerMotorInverted,
                kBackLeftCANcoderInverted);
        public static final SwerveModuleConstants BackRight =
            ConstantCreator.createModuleConstants(
                kBackRightSteerMotorId,
                kBackRightDriveMotorId,
                kBackRightEncoderId,
                kBackRightEncoderOffset,
                kBackRightXPos,
                kBackRightYPos,
                kInvertRightSide,
                kBackRightSteerMotorInverted,
                kBackRightCANcoderInverted);

        // /**
        //  * Creates a CommandSwerveDrivetrain instance.
        //  * This should only be called once in your robot program,.
        //  */
        // public static CommandSwerveDrivetrain createDrivetrain() {
        //     return new CommandSwerveDrivetrain(
        //         DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        //     );
        // }
    
}
