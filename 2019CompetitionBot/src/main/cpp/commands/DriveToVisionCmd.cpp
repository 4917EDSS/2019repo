/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveToVisionCmd.h"
#include "Robot.h"

DriveToVisionCmd::DriveToVisionCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::drivetrainSub);
}

// Called just before this Command runs the first time
void DriveToVisionCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveToVisionCmd::Execute() {
  double xAngle = Robot::GetVisionTarget();
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);


 if (xAngle > 9000){
   Robot::drivetrainSub.drive(0.0, 0.0);
 }
 else {
   Robot::drivetrainSub.drive( 0.5 + xAngle, 0.5 - xAngle);
 }
 
}

// Make this return true when this Command no longer needs to run execute()
bool DriveToVisionCmd::IsFinished() { 
  double xAngle = Robot::GetVisionTarget();

  if (-1 < xAngle < 1){
    return false;
  }
  else {
    return false;
  }
}

// Called once after isFinished returns true
void DriveToVisionCmd::End() {
  Robot::drivetrainSub.drive(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveToVisionCmd::Interrupted() {
  DriveToVisionCmd::End();
}
