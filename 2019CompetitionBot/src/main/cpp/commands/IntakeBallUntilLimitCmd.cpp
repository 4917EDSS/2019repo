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
}

// Called just before this Command runs the first time
void IntakeBallUntilLimitCmd::Initialize() {
  Robot::ballIntakeSub.SetIntakeMotor(1.0);
}

// Called repeatedly when this Command is scheduled to run
void IntakeBallUntilLimitCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool IntakeBallUntilLimitCmd::IsFinished() {
  return Robot::ballIntakeSub.isBallIn();
 }

// Called once after isFinished returns true
void IntakeBallUntilLimitCmd::End() {
  Robot::ballIntakeSub.SetIntakeMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void IntakeBallUntilLimitCmd::Interrupted() {
  IntakeBallUntilLimitCmd::End();
}
