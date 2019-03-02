/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetManipulatorCmd.h"
#include "Robot.h"

SetManipulatorCmd::SetManipulatorCmd(double targetAngle) : targetAngle(targetAngle) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
    Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void SetManipulatorCmd::Initialize() {
  Robot::manipulatorSub.setFlipperAngle(FLIPPER_MODE_AUTO, 1.0, targetAngle);
}

// Called repeatedly when this Command is scheduled to run
void SetManipulatorCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetManipulatorCmd::IsFinished() { 
  return Robot::manipulatorSub.isFlipperAtTarget(); 
}

// Called once after isFinished returns true
void SetManipulatorCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetManipulatorCmd::Interrupted() {
  End();
}
