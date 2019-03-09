/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FoldIntakeCmd.h"
#include "Robot.h"

FoldIntakeCmd::FoldIntakeCmd(bool foldIn) : foldIn(foldIn) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);
}

// Called just before this Command runs the first time
void FoldIntakeCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s | foldIn = %d\n", __FILE__, __FUNCTION__, foldIn);
 
  if(foldIn) {
    Robot::ballIntakeSub.foldIntakeArms();
  }
  else {
    Robot::ballIntakeSub.unfoldIntakeArms();
  }
}

// Called repeatedly when this Command is scheduled to run
void FoldIntakeCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool FoldIntakeCmd::IsFinished() { 
  if(TimeSinceInitialized() >= 0.5) {
    return true;
  }
  return false; 
}

// Called once after isFinished returns true
void FoldIntakeCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FoldIntakeCmd::Interrupted() {
  End();
}
