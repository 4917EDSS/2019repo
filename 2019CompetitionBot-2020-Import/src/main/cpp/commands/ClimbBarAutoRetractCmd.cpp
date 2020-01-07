/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimbBarAutoRetractCmd.h"
#include "Robot.h"
#include "subsystems/ClimbSub.h"

ClimbBarAutoRetractCmd::ClimbBarAutoRetractCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::climbSub);
}

// Called just before this Command runs the first time
void ClimbBarAutoRetractCmd::Initialize() {
  if(!IsFinished()) {
    Robot::climbSub.SetClimbMotorPower(-1.0);
  }
}

// Called repeatedly when this Command is scheduled to run
void ClimbBarAutoRetractCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool ClimbBarAutoRetractCmd::IsFinished() { 
  if(Robot::climbSub.getClimbPosition() <= CLIMB_RETRACT_LIMIT_THRESHOLD) {
    return true;
  }
  else {
    return false; 
  }
 }

// Called once after isFinished returns true
void ClimbBarAutoRetractCmd::End() {
  Robot::climbSub.SetClimbMotorPower(0.0);
  Robot::inClimbMode = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClimbBarAutoRetractCmd::Interrupted() {
  End();
}
