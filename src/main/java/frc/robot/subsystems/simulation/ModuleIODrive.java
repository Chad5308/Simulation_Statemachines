package frc.robot.subsystems.simulation;

import static frc.robot.subsystems.drive.DriveConstants.kBackLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightDriveInverted;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftDriveInverted;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftZeroRotation;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightDriveInverted;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftSteerMotorInverted;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightSteerMotorInverted;
import static frc.robot.subsystems.drive.DriveConstants.kBackLeftSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackLeftSteerMotorInverted;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightSteerMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightSteerMotorInverted;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftDriveMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontLeftEncoderId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightDriveMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kFrontRightEncoderId;
import static frc.robot.subsystems.drive.DriveConstants.kBackLeftDriveInverted;
import static frc.robot.subsystems.drive.DriveConstants.kBackLeftDriveMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackLeftEncoderId;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightDriveMotorId;
import static frc.robot.subsystems.drive.DriveConstants.kBackRightEncoderId;

import static frc.robot.util.DriveUtil.talonUtil.*;
import static frc.robot.util.DriveUtil.sparkUtil.*;


import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
   private final Rotation2d zeroRotation;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkBase turnSpark;
  private final AbsoluteEncoder turnEncoder;
  private final CANcoder cancoder;

    // Voltage control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    // private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Closed loop controllers
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;

  // Queue inputs from odometry thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final StatusSignal<Angle> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> turnCurrent;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

  public ModuleIODrive(int module) {
    cancoder =
        new CANcoder(
            switch (module) {
                case 0 -> kFrontLeftEncoderId;
                case 1 -> kFrontRightEncoderId;
                case 2 -> kBackLeftEncoderId;
                case 3 -> kBackRightEncoderId;
                default -> 0;
            }
        );
    zeroRotation =
        switch (module) {
          case 0 -> kFrontLeftZeroRotation;
          case 1 -> kFrontRightZeroRotation;
          case 2 -> kBackLeftZeroRotation;
          case 3 -> kBackRightZeroRotation;
          default -> new Rotation2d();
        };
    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> kFrontLeftDriveMotorId;
              case 1 -> kFrontRightDriveMotorId;
              case 2 -> kBackLeftDriveMotorId;
              case 3 -> kBackRightDriveMotorId;
              default -> 0;
            }, DriveConstants.kCANBus);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> kFrontLeftSteerMotorId;
              case 1 -> kFrontRightSteerMotorId;
              case 2 -> kBackLeftSteerMotorId;
              case 3 -> kBackRightSteerMotorId;
              default -> 0;
            },
            MotorType.kBrushless);
    // driveEncoder = driveSpark.getEncoder();
    turnEncoder = turnSpark.getAbsoluteEncoder();
    // driveController = driveSpark.getClosedLoopController();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    var driveConfig = DriveConstants.driveInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = DriveConstants.driveGains;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = DriveConstants.slipCurrent.magnitude();
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -DriveConstants.slipCurrent.magnitude();
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.slipCurrent.magnitude();
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        switch(module) {
          case 0 -> kFrontLeftDriveInverted;
          case 1 -> kFrontRightDriveInverted;
          case 2 -> kBackLeftDriveInverted;
          case 3 -> kBackRightDriveInverted;
          default -> false;
        }
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    // talonUtil.tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    // talonUtil.tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));
    driveTalon.getConfigurator().apply(driveConfig, 0.25);
    driveTalon.setPosition(0.0, 0.25);

    // Configure turn motor
    var turnConfig = new SparkMaxConfig();
    turnConfig
        .inverted(
          switch(module) {
            case 0 -> kFrontLeftSteerMotorInverted;
            case 1 -> kFrontRightSteerMotorInverted;
            case 2 -> kBackLeftSteerMotorInverted;
            case 3 -> kBackRightSteerMotorInverted;
            default -> false;
          }
        )
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(DriveConstants.turnMotorCurrentLimit)
        .voltageCompensation(12.0);
    turnConfig
        .encoder
        .positionConversionFactor(DriveConstants.turnEncoderPositionFactor)
        .velocityConversionFactor(DriveConstants.turnEncoderVelocityFactor)
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(DriveConstants.turnPIDMinInput, DriveConstants.turnPIDMaxInput)
        .pidf(DriveConstants.turnKp, 0.0, DriveConstants.turnKd, 0.0);
    turnConfig
        .signals
        .absoluteEncoderPositionAlwaysOn(true)
        .absoluteEncoderPositionPeriodMs((int) (1000.0 / DriveConstants.odometryFrequency.getValue()))
        .absoluteEncoderVelocityAlwaysOn(true)
        .absoluteEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);
    // tryUntilOk(
    //     turnSpark,
    //     5,
    //     () ->
    //         turnSpark.configure(
    //             turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    // Create odometry queues
    timestampQueue = DriveOdometry.getInstance().makeTimestampQueue();
       
    // Create drive status signals
      drivePosition = driveTalon.getPosition();
      drivePositionQueue =
      DriveOdometry.getInstance().registerTalonSignal(driveTalon.getPosition());
      driveVelocity = driveTalon.getVelocity();
      driveAppliedVolts = driveTalon.getMotorVoltage();
      driveCurrent = driveTalon.getStatorCurrent();

    turnPositionQueue =
        DriveOdometry.getInstance().registerSparkSignal(turnSpark, turnEncoder::getPosition);
  }




  @Override
  public void updateInputs(ModuleIOInputs inputs) {
  
    // Refresh all signals
    var driveStatus =
    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

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



  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageRequest.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentRequest.withOutput(output);
        });
  }

    
  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltageRequest.withVelocity(velocityRotPerSec);
          case TorqueCurrentFOC -> velocityTorqueCurrentRequest.withVelocity(velocityRotPerSec);
        });
  }


    
  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(
            rotation.plus(zeroRotation).getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setReference(setpoint, ControlType.kPosition);
  }



}
