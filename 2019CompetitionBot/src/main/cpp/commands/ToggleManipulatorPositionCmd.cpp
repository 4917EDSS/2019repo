/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ToggleManipulatorPositionCmd.h"
#include "subsystems/elevatorSub.h"
#include "Robot.h"

ToggleManipulatorPositionCmd::ToggleManipulatorPositionCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void ToggleManipulatorPositionCmd::Initialize() {
  double targetAngle;

  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);

  if(Robot::manipulatorSub.getFlipperTargetAngle() > 0) {
    targetAngle = -90;
  }
  else {
    targetAngle = 90;
  }

  Robot::manipulatorSub.setFlipperAngle(FLIPPER_MODE_AUTO, 0.5, targetAngle);
}

// Called repeatedly when this Command is scheduled to run
void ToggleManipulatorPositionCmd::Execute() {
  
}

// Make this return true when this Command no longer needs to run execute()
bool ToggleManipulatorPositionCmd::IsFinished() { 
  return Robot::manipulatorSub.isFlipperAtTarget(); 
}

// Called once after isFinished returns true
void ToggleManipulatorPositionCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ToggleManipulatorPositionCmd::Interrupted() {
  End();
}
