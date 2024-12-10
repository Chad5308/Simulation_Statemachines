// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VelocityUnit;
// import frc.robot.Constants.constField;
// import frc.robot.Constants.constStateMachine;
// import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.drive.Drive;

public class AutoDriveState extends Command {
  StateMachine s_StateMachine;
  Drive s_Drive;

  Measure<VelocityUnit<AngleUnit>> desiredLeftVelocity, desiredRightVelocity;
  Pose3d[] fieldPoses;
  Pose2d robotPose = new Pose2d();

  public AutoDriveState(StateMachine s_StateMachine, Drive s_Drive) {
    this.s_StateMachine = s_StateMachine;
    this.s_Drive = s_Drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_StateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState currentRobotState = s_StateMachine.getRobotState();

    // if (currentRobotState.equals(RobotState.STORE_FEEDER) || s_StateMachine.isCurrentStateTargetState()) {
    //   s_StateMachine.setRobotState(RobotState.PREP_VISION);
    // }TODO

    // fieldPoses = constField.getFieldPositions().get();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // robotPose = s_Drive.getPose();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
