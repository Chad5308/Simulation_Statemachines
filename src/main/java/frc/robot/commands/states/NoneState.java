// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.constElevator;
// import frc.robot.Constants.constLEDs;
// import frc.robot.Constants.constShooter;
// import frc.robot.Constants.constStateMachine;
// import frc.robot.Constants.constShooter.ShooterPositionGroup;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.StateMachine.RobotState;
import frc.robot.subsystems.StateMachine.TargetState;


public class NoneState extends Command {
  StateMachine s_StateMachine;
  Lights s_Lights;

  /** Creates a new NoneState. */
  public NoneState(StateMachine s_StateMachine, Lights s_Lights) {
    this.s_StateMachine = s_StateMachine;
    this.s_Lights = s_Lights;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_StateMachine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {

    s_StateMachine.setTargetState(TargetState.PREP_NONE);
    s_StateMachine.setRobotState(RobotState.NONE);
    // subLEDs.clearAnimation();
    // subLEDs.setLEDs(constLEDs.CLEAR_LEDS);

    // if (subShooter.isSafeToMoveElevator()) {
    //   subShooter.setPivotNeutralOutput();
    // } else {
    //   subShooter.setPivotPosition(constShooter.NONE_STATE_ANGLE);
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // subShooter.setPivotNeutralOutput();
    // subElevator.setElevatorPosition(constElevator.BACKWARD_LIMIT);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return subShooter.isSafeToMoveElevator();
    return false;
  }
}
