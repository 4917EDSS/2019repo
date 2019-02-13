/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetIntakeArmAngleCmd.h"
#include "Robot.h"

SetIntakeArmAngleCmd::SetIntakeArmAngleCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);
}

// Called just before this Command runs the first time
void SetIntakeArmAngleCmd::Initialize() {
  //These values need testing
  Robot::ballIntakeSub.setFlipperOut(true);
  Robot::ballIntakeSub.setIntakeArmMotor(0.25);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeArmAngleCmd::Execute() {
  Robot::ballIntakeSub.update();
}

// Make this return true when this Command no longer needs to run execute()
bool SetIntakeArmAngleCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void SetIntakeArmAngleCmd::End() {
  Robot::ballIntakeSub.setFlipperOut(false);
  Robot::ballIntakeSub.setIntakeArmMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetIntakeArmAngleCmd::Interrupted() {}
