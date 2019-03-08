/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimbReverseCmd.h"
#include "Robot.h"

ClimbReverseCmd::ClimbReverseCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::climbSub);
  Requires(&Robot::ballIntakeSub);
}

// Called just before this Command runs the first time
void ClimbReverseCmd::Initialize() {
  Robot::climbSub.SetClimbMotorPower(-1.0);
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_AUTO, 0.5, INTAKE_NEUTRAL_ANGLE);
}

// Called repeatedly when this Command is scheduled to run
void ClimbReverseCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool ClimbReverseCmd::IsFinished() { 
  if(Robot::climbSub.getClimbPosition() <= CLIMB_RETRACT_LIMIT_THRESHOLD) {
    return true;
  }
  else {
    return false; 
  }
}

// Called once after isFinished returns true
void ClimbReverseCmd::End() {
  Robot::climbSub.SetClimbMotorPower(0.0);
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_MANUAL, 0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClimbReverseCmd::Interrupted() {}
