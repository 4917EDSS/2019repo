/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FoldIntakeImmediateCmd.h"
#include "Robot.h"

FoldIntakeImmediateCmd::FoldIntakeImmediateCmd(bool foldIn) : foldIn(foldIn) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);

}

// Called just before this Command runs the first time
void FoldIntakeImmediateCmd::Initialize() {
   logger.send(logger.CMD_TRACE, "%s : %s | foldIn = %d\n", __FILE__, __FUNCTION__, foldIn);
 
  if(foldIn) {
    Robot::ballIntakeSub.foldIntakeArms();
  }
  else {
    Robot::ballIntakeSub.unfoldIntakeArms();
  }
}

// Called repeatedly when this Command is scheduled to run
void FoldIntakeImmediateCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool FoldIntakeImmediateCmd::IsFinished() { return true; }

// Called once after isFinished returns true
void FoldIntakeImmediateCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FoldIntakeImmediateCmd::Interrupted() {
  End();
}
