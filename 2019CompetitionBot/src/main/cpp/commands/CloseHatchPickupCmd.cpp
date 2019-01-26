/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/CloseHatchPickupCmd.h"
#include "robot.h"

CloseHatchPickupCmd::CloseHatchPickupCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void CloseHatchPickupCmd::Initialize() {
  Robot::manipulatorSub.ContractHatchGripper();
}
// Called repeatedly when this Command is scheduled to run
void CloseHatchPickupCmd::Execute() {
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);
}

// Make this return true when this Command no longer needs to run execute()
bool CloseHatchPickupCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void CloseHatchPickupCmd::End() {
  Robot::manipulatorSub.ExpandHatchGripper();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CloseHatchPickupCmd::Interrupted() {
  End();
}
