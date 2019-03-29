/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DoNothingHatchCmd.h"
#include "Robot.h"


DoNothingHatchCmd::DoNothingHatchCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void DoNothingHatchCmd::Initialize() {
      Robot::inBallMode = false;

}

// Called repeatedly when this Command is scheduled to run
void DoNothingHatchCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool DoNothingHatchCmd::IsFinished() { return true; }

// Called once after isFinished returns true
void DoNothingHatchCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DoNothingHatchCmd::Interrupted() {}
