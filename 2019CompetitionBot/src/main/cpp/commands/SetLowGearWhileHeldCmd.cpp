/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetLowGearWhileHeldCmd.h"
#include "Robot.h"

SetLowGearWhileHeldCmd::SetLowGearWhileHeldCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void SetLowGearWhileHeldCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::elevatorSub.setShifterHigh(false);
}

// Called repeatedly when this Command is scheduled to run
void SetLowGearWhileHeldCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetLowGearWhileHeldCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void SetLowGearWhileHeldCmd::End() {
  Robot::elevatorSub.setShifterHigh(true);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetLowGearWhileHeldCmd::Interrupted() {
  End();
}
