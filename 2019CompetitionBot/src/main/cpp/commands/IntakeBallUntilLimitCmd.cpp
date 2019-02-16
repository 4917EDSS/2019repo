/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeBallUntilLimitCmd.h"
#include "Robot.h"

IntakeBallUntilLimitCmd::IntakeBallUntilLimitCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void IntakeBallUntilLimitCmd::Initialize() {
  Robot::ballIntakeSub.setIntakeMotor(1.0);
  Robot::elevatorSub.setManipulatorWheelSpeed(-0.5, -0.5);
}

// Called repeatedly when this Command is scheduled to run
void IntakeBallUntilLimitCmd::Execute() {
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);
}

// Make this return true when this Command no longer needs to run execute()
bool IntakeBallUntilLimitCmd::IsFinished() {
  return Robot::elevatorSub.isBallInManipulator();
 }

// Called once after isFinished returns true
void IntakeBallUntilLimitCmd::End() {
  Robot::ballIntakeSub.setIntakeMotor(0.0);
  Robot::elevatorSub.setManipulatorWheelSpeed(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void IntakeBallUntilLimitCmd::Interrupted() {
  IntakeBallUntilLimitCmd::End();
}
