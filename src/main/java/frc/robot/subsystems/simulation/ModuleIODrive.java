package frc.robot.subsystems.simulation;

import static frc.robot.subsystems.drive.DriveConstants.kBackLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackLeftSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftDriveMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightDriveMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackLeftDriveMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightDriveMotorId;

import static frc.robot.util.DriveUtil.*;


import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveOdometry;
import frc.robot.subsystems.simulation.ModuleIO.ModuleIOInputs;
import frc.robot.util.DriveUtil;
import frc.robot.util.DriveUtil.talonUtil;

public class ModuleIODrive implements ModuleIO
{
    private final SwerveModuleConstants constants;
    private final Rotation2d zeroRotation;

    // Hardware objects
    private final TalonFX driveTalon;
    private final SparkBase turnNEO;
    private final CANcoder cancoder;

    // Closed loop controllers
    private final SparkClosedLoopController turnController;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // Torque-current control requests
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
        new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
        new VelocityTorqueCurrentFOC(0.0);

    // Queue inputs from Odometry
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // // Inputs from turn motor
    // private final StatusSignal<Angle> turnAbsolutePosition;
    // private final StatusSignal<Angle> turnPosition;
    // private final StatusSignal<AngularVelocity> turnVelocity;
    // private final StatusSignal<Voltage> turnAppliedVolts;
    // private final StatusSignal<Current> turnCurrent;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
    private final Debouncer cancoderConnectedDebounce = new Debouncer(0.5);

    public ModuleIODrive(SwerveModuleConstants constants, int module)
    {
        
        zeroRotation =
        switch (module)
        {
            case 0 -> kFrontLeftZeroRotation;
            case 1 -> kFrontRightZeroRotation;
            case 2 -> kBackLeftZeroRotation;
            case 3 -> kBackRightZeroRotation;
            default -> new Rotation2d();
        };

        driveTalon =
        new TalonFX(
            switch (module)
            {
                case 0 -> kFrontLeftDriveMotorId;
                case 1 -> kFrontRightDriveMotorId;
                case 2 -> kBackLeftDriveMotorId;
                case 3 -> kBackRightDriveMotorId;
                default -> 0;
            }, DriveConstants.kCANBus.getName());
        turnNEO =
        new SparkMax(
            switch (module)
            {
                case 0 -> kFrontLeftSteerMotorId;
                case 1 -> kFrontRightSteerMotorId;
                case 2 -> kBackLeftSteerMotorId;
                case 3 -> kBackRightSteerMotorId;
                default -> 0;
            }, MotorType.kBrushless);
                
                
                
        this.constants = constants;
        turnController = turnNEO.getClosedLoopController();
        cancoder = new CANcoder(constants.CANcoderId, DriveConstants.DrivetrainConstants.CANBusName);
        
        //Cnfig drive motor
        TalonFXConfiguration driveConfig = constants.DriveMotorInitialConfigs;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = constants.DriveMotorGains;
        driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.MotorOutput.Inverted =
            constants.DriveMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        talonUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
        talonUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));


         // Configure CANCoder
        CANcoderConfiguration cancoderConfig = constants.CANcoderInitialConfigs;
        cancoderConfig.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection =
            constants.CANcoderInverted
                ? SensorDirectionValue.Clockwise_Positive
                : SensorDirectionValue.CounterClockwise_Positive;
        cancoder.getConfigurator().apply(cancoderConfig);

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
            .inverted(turnInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(turnMotorCurrentLimit)
            .voltageCompensation(12.0);
        turnConfig
            .absoluteEncoder
            .inverted(cancoderInverted)
            .positionConversionFactor(cancoderPositionFactor)
            .velocityConversionFactor(cancoderVelocityFactor)
            .averageDepth(2);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
            .pidf(turnKp, 0.0, turnKd, 0.0);
        turnConfig
            .signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency))
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        tryUntilOk(
            turnNEO,
            5,
            () ->
                turnNEO.configure(
                    turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));



         // Create timestamp queue
        timestampQueue = DriveOdometry.PhoenixOdometry.getInstance().makeTimestampQueue();


    }


  @Override
  public void updateInputs(ModuleIOInputs inputs) {
        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        drivePositionQueue =
        DriveOdometry.PhoenixOdometry.getInstance().registerSignal(driveTalon.getPosition());
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();


         // Update turn inputs
        DriveUtil.sparkUtil.sparkStickyFault = false;
        ifOk(
            turnNEO,
            cancoder::getPosition,
            (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnNEO, cancoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
            turnNEO,
            new DoubleSupplier[] {turnNEO::getAppliedOutput, turnNEO::getBusVoltage},
            (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnNEO, turnNEO::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!DriveUtil.sparkUtil.sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }




}
