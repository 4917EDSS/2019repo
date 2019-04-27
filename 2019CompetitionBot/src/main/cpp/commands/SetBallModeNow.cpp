/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetBallModeNow.h"
#include "Robot.h"

SetBallModeNow::SetBallModeNow() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void SetBallModeNow::Initialize() {
Robot::inBallMode = true;
}

// Called repeatedly when this Command is scheduled to run
void SetBallModeNow::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetBallModeNow::IsFinished() { return true; }

// Called once after isFinished returns true
void SetBallModeNow::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetBallModeNow::Interrupted() {}
