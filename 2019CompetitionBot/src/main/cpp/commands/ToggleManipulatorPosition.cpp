/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ToggleManipulatorPosition.h"
#include "subsystems/elevatorSub.h"
#include "Robot.h"

ToggleManipulatorPosition::ToggleManipulatorPosition() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void ToggleManipulatorPosition::Initialize() {
  if (Robot::elevatorSub.getManipulatorAngle() == -90) {
    Robot::elevatorSub.setManipulatorTargetAngle(90);
  }
  else if (Robot::elevatorSub.getManipulatorAngle() == 90){
Robot::elevatorSub.setManipulatorTargetAngle(-90);
  }
}

// Called repeatedly when this Command is scheduled to run
void ToggleManipulatorPosition::Execute() {
  
}

// Make this return true when this Command no longer needs to run execute()
bool ToggleManipulatorPosition::IsFinished() { return false; }

// Called once after isFinished returns true
void ToggleManipulatorPosition::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ToggleManipulatorPosition::Interrupted() {}
