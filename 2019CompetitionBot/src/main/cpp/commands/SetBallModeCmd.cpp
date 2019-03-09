/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetBallModeCmd.h"
#include "Robot.h"



SetBallModeCmd::SetBallModeCmd() {
  Requires(&Robot::manipulatorSub);
}

void SetBallModeCmd::Initialize() {
    Robot::manipulatorSub.setIntakePower(-0.5);
    Robot::inBallMode = true;
}

// Called repeatedly when this Command is scheduled to run
void SetBallModeCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetBallModeCmd::IsFinished() { 
  if (TimeSinceInitialized() >= 1.5) {
    return true;
  }
  return false;
}

// Called once after isFinished returns true
void SetBallModeCmd::End() {
    Robot::manipulatorSub.setIntakePower(0);

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetBallModeCmd::Interrupted() {
  End();
}
