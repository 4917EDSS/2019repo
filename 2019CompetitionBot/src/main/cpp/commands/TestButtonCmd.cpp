/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TestButtonCmd.h"
#include "Robot.h"

TestButtonCmd::TestButtonCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);
}

// Called just before this Command runs the first time
void TestButtonCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::ballIntakeSub.setIntakeWheelPower(1.0);
  Robot::ballIntakeSub.unfoldIntakeArms();
}

// Called repeatedly when this Command is scheduled to run
void TestButtonCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool TestButtonCmd::IsFinished() { return false; }


void TestButtonCmd::End() {
  Robot::ballIntakeSub.setIntakeWheelPower(0.0);
  Robot::ballIntakeSub.foldIntakeArms();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestButtonCmd::Interrupted() {
  End();
}
