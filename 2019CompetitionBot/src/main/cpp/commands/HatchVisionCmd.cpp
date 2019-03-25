/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/HatchVisionCmd.h"
#include "OI.H"
#include "Robot.h"
#include <iostream>

HatchVisionCmd::HatchVisionCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::drivetrainSub);
}

// Called just before this Command runs the first time
void HatchVisionCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::visionSub.setManipulatorPipeline(VISION_MODE_NORMAL);
}

// Called repeatedly when this Command is scheduled to run
void HatchVisionCmd::Execute() {
  double targetAngle=Robot::visionSub.getVisionTarget(BUMPER_CAMERA);
  double robotAngle=Robot::drivetrainSub.getAngle();
  double scoringFace=Robot::visionSub.getScoringFaceAngle(BUMPER_CAMERA);
  double robotTargetAngle=Robot::visionSub.getRobotTargetAngle(robotAngle, targetAngle, scoringFace);

  if (Robot::visionSub.isTargetVisible(BUMPER_CAMERA) ){
    double lSpeed=(0.3+(targetAngle*0.007));
    double rSpeed=(0.3-(targetAngle*0.007));
    Robot::drivetrainSub.drive(lSpeed,rSpeed);
  } else {
    Robot::drivetrainSub.drive(0,0);    
  } 
  

}

// Make this return true when this Command no longer needs to run execute()
bool HatchVisionCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void HatchVisionCmd::End() {
  Robot::drivetrainSub.drive(0,0);
  Robot::visionSub.setManipulatorPipeline(DRIVER_MODE_NORMAL);

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void HatchVisionCmd::Interrupted() {
  End();
}
