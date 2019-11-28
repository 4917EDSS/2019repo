/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/GripperToggleCmd.h"
#include "robot.h"

GripperToggleCmd::GripperToggleCmd() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void GripperToggleCmd::Initialize() {
  if(Robot::manipulatorSub.isGripperExpanded()) {
    Robot::manipulatorSub.contractHatchGripper();
  }
  else {
    Robot::manipulatorSub.expandHatchGripper();
  }
}

// Called repeatedly when this Command is scheduled to run
void GripperToggleCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool GripperToggleCmd::IsFinished() { return true; }

// Called once after isFinished returns true
void GripperToggleCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GripperToggleCmd::Interrupted() {}
