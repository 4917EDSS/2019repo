/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetManipulatorAngleCmd.h"
#include "Robot.h"

SetManipulatorAngleCmd::SetManipulatorAngleCmd(double targetAngle) : targetAngle(targetAngle) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
    Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void SetManipulatorAngleCmd::Initialize() {
  Robot::manipulatorSub.setFlipperAngle(FLIPPER_MODE_AUTO, 1.0, targetAngle);
}

// Called repeatedly when this Command is scheduled to run
void SetManipulatorAngleCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetManipulatorAngleCmd::IsFinished() { 
  return Robot::manipulatorSub.isFlipperAtTarget(); 
}

// Called once after isFinished returns true
void SetManipulatorAngleCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetManipulatorAngleCmd::Interrupted() {
  End();
}
