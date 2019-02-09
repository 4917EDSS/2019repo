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
  Requires(&Robot::drivetrainSub);
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void TestButtonCmd::Initialize() {
   Robot::ballIntakeSub.setIntakeArmMotor(0.05);
}

// Called repeatedly when this Command is scheduled to run
void TestButtonCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool TestButtonCmd::IsFinished() { return false; }


void TestButtonCmd::End() {
  Robot::ballIntakeSub.setIntakeArmMotor(0.0);

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestButtonCmd::Interrupted() {
  End();
}
