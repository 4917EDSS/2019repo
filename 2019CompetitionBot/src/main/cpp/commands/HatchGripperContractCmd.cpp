/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/HatchGripperContractCmd.h"
#include "robot.h"

HatchGripperContractCmd::HatchGripperContractCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void HatchGripperContractCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::manipulatorSub.contractHatchGripper();
}
// Called repeatedly when this Command is scheduled to run
void HatchGripperContractCmd::Execute() {
}

// Make this return true when this Command no longer needs to run execute()
bool HatchGripperContractCmd::IsFinished() {
  if(TimeSinceInitialized() >= 0.25) {
    return true;
}
  return false;
}

// Called once after isFinished returns true
void HatchGripperContractCmd::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void HatchGripperContractCmd::Interrupted() {
  End();
}
