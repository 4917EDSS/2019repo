/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/YeetCmd.h"
#include "Robot.h"

YeetCmd::YeetCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  
  Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void YeetCmd::Initialize() {
  Robot::ballIntakeSub.setIntakeWheelPower(1.0);
}

// Called repeatedly when this Command is scheduled to run
void YeetCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool YeetCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void YeetCmd::End() {
  Robot::ballIntakeSub.setIntakeWheelPower(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void YeetCmd::Interrupted() { End(); }
