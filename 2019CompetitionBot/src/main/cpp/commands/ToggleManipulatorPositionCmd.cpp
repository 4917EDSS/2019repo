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
  Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void ToggleManipulatorPositionCmd::Initialize() {
  double targetAngle;
  int shift = Robot::oi.getOperatorShiftState();

  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);

  switch(shift) {  
    
    case 1:
      // Shift-Up:  Rotate to vertical
      targetAngle = 0;
      break;

    case 2:
      // Shift-Right:  Rotate to 90 deg forwards
      targetAngle = 90;
      break;

    case 4:
      // Shift-Left:  Rotate to 90 deg backwards
      targetAngle = -90;
      break;
    
    case 0:
    default:
      // Toggle between 90 deg forwards or 90 deg backwards
      if(Robot::manipulatorSub.getFlipperTargetAngle() >= -5 && Robot::manipulatorSub.getFlipperTargetAngle() <= 5) {
        targetAngle = 90;
      }
      else if(Robot::manipulatorSub.getFlipperTargetAngle() > 5 ) {
        targetAngle = -90;
      }
      else {
        targetAngle = 90;
      }
      break;
  }

  Robot::manipulatorSub.setFlipperAngle(FLIPPER_MODE_AUTO, 1.0, targetAngle);
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
