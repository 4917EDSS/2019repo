/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DynamicElevatorHeightCnd.h"
#include "Robot.h"


DynamicElevatorHeightCnd::DynamicElevatorHeightCnd(Command *onTrue, Command *onFalse) : frc::ConditionalCommand(onTrue, onFalse) {

}


// First parameter (onTrue) is hatch height.  Second parameter (onFalse) is cargo height
bool DynamicElevatorHeightCnd::Condition() {
  return !Robot::inBallMode;
}

// Called just before this Command runs the first time
void DynamicElevatorHeightCnd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DynamicElevatorHeightCnd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool DynamicElevatorHeightCnd::IsFinished() { return false; }

// Called once after isFinished returns true
void DynamicElevatorHeightCnd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DynamicElevatorHeightCnd::Interrupted() {}
