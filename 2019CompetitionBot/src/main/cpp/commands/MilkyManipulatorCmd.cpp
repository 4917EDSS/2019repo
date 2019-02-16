/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MilkyManipulatorCmd.h"
#include "OI.H"
#include "Robot.h"
#include "RobotPathHelpers.h"
#include <iostream>

MilkyManipulatorCmd::MilkyManipulatorCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void MilkyManipulatorCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MilkyManipulatorCmd::Execute() {
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);

  double targetAngle=Robot::GetVisionTarget();
  double distance=Robot::GetDistanceFromVision();
  double robotAngle=Robot::drivetrainSub.getAngle();
  double scoringFace=Robot::GetScoringFaceAngle();
  double robotTargetAngle=GetRobotTargetAngle(robotAngle, targetAngle, distance, scoringFace);

  double lSpeed=(0.5+(robotAngle-robotTargetAngle)* 0.1);
  double rSpeed=(0.5-(robotAngle-robotTargetAngle)* 0.1);

  Robot::drivetrainSub.drive(lSpeed,rSpeed);
}

// Make this return true when this Command no longer needs to run execute()
bool MilkyManipulatorCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void MilkyManipulatorCmd::End() {
  //MilkyManipulatorCmd.milkyManipulator(0.0);
  Robot::drivetrainSub.drive(0,0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MilkyManipulatorCmd::Interrupted() {
  End();
}
