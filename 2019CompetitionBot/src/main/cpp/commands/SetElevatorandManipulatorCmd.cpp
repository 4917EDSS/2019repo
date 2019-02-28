/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetElevatorandManipulatorCmd.h"
#include "robot.h"

SetElevatorandManipulatorCmd::SetElevatorandManipulatorCmd(double targetAngle, double targetHeight) : targetAngle(targetAngle), targetHeight(targetHeight){
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void SetElevatorandManipulatorCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::elevatorSub.setElevatorHeight(ELEVATOR_MODE_AUTO, 0.5, targetHeight);
  Robot::manipulatorSub.setFlipperAngle(FLIPPER_MODE_AUTO, 0.5, targetAngle);
}

// Called repeatedly when this Command is scheduled to run
void SetElevatorandManipulatorCmd::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool SetElevatorandManipulatorCmd::IsFinished() { 
  return Robot::elevatorSub.isElevatorAtTarget() && Robot::manipulatorSub.isFlipperAtTarget();
}

// Called once after isFinished returns true
void SetElevatorandManipulatorCmd::End() {
  
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetElevatorandManipulatorCmd::Interrupted() {
  End();
}
