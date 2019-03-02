/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetElevatorToHeightCmd.h"
#include "Robot.h"

SetElevatorToHeightCmd::SetElevatorToHeightCmd(double height) : height(height) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void SetElevatorToHeightCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s | Height = %.1f\n", __FILE__, __FUNCTION__), height;
  Robot::elevatorSub.setElevatorHeight(ELEVATOR_MODE_AUTO, 1.0, height);
}

// Called repeatedly when this Command is scheduled to run
void SetElevatorToHeightCmd::Execute() {

}

// Make this return true when this Command no longer needs to run execute()
bool SetElevatorToHeightCmd::IsFinished() {
  return Robot::elevatorSub.isElevatorAtTarget();
}

// Called once after isFinished returns true
void SetElevatorToHeightCmd::End() {
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetElevatorToHeightCmd::Interrupted() {
  End();
}
