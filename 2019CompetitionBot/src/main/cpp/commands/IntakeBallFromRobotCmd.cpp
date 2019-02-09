/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeBallFromRobotCmd.h"
#include "subsystems/BallIntakeSub.h"
#include "Robot.h"

IntakeBallFromRobotCmd::IntakeBallFromRobotCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);
}

// Called just before this Command runs the first time
void IntakeBallFromRobotCmd::Initialize() {
  //Robot::ballIntakeSub.setWheels(0.5, 0.5);
}

// Called repeatedly when this Command is scheduled to run
void IntakeBallFromRobotCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool IntakeBallFromRobotCmd::IsFinished() { 
  //return Robot::ballIntakeSub.isBallInManipulator();
}

// Called once after isFinished returns true
void IntakeBallFromRobotCmd::End() {
  //Robot::ballIntakeSub.setWheels(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void IntakeBallFromRobotCmd::Interrupted() {}