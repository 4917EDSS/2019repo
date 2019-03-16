/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/KillEverythingCmd.h"
#include "Robot.h"

KillEverythingCmd::KillEverythingCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);
  Requires(&Robot::drivetrainSub);
  Requires(&Robot::elevatorSub);
  Requires(&Robot::manipulatorSub);
  Requires(&Robot::climbSub);
}

// Called just before this Command runs the first time
void KillEverythingCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
}

// Called repeatedly when this Command is scheduled to run
void KillEverythingCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool KillEverythingCmd::IsFinished() {  
 return true; 
}

// Called once after isFinished returns true
void KillEverythingCmd::End() {
  Robot::inClimbMode = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void KillEverythingCmd::Interrupted() {}
