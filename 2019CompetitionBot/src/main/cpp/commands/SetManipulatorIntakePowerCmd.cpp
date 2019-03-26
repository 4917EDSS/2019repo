/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "commands/SetManipulatorIntakePowerCmd.h"

SetManipulatorIntakePowerCmd::SetManipulatorIntakePowerCmd(double power):power(power) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void SetManipulatorIntakePowerCmd::Initialize() {
  Robot::manipulatorSub.setIntakePower(power);
}

// Called repeatedly when this Command is scheduled to run
void SetManipulatorIntakePowerCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetManipulatorIntakePowerCmd::IsFinished() { return true; }

// Called once after isFinished returns true
void SetManipulatorIntakePowerCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetManipulatorIntakePowerCmd::Interrupted() {}
