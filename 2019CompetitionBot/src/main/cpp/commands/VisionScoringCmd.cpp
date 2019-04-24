/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/VisionScoringCmd.h"
#include "OI.H"
#include "Robot.h"
#include <iostream>

constexpr double JOYSTICK_DEADBAND = 0.01;
double maxWidth = 120;
VisionScoringCmd::VisionScoringCmd(): VisionScoringCmd(false) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}
VisionScoringCmd::VisionScoringCmd(bool driveAllTheWay): driveAllTheWay(driveAllTheWay){
  Requires(&Robot::drivetrainSub);

}

// Called just before this Command runs the first time
void VisionScoringCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  noLongerSeesTarget = false;
  Robot::visionSub.setBumperPipeline(VISION_MODE_NORMAL);
  timeSinceTargetSeen = 9999999; 
}

// Called repeatedly when this Command is scheduled to run
void VisionScoringCmd::Execute() {
  
  double targetAngle=Robot::visionSub.getVisionTarget(BUMPER_CAMERA);
  double verticalOffset = Robot::visionSub.getVerticalOffset(BUMPER_CAMERA);

  if (Robot::visionSub.isTargetVisible(BUMPER_CAMERA) && (verticalOffset < 13.00) && !noLongerSeesTarget) {
    double lSpeed=(0);
    double rSpeed=(0);
    double percent;
    double difference;
    percent = Robot::visionSub.getHorizontalWidth(BUMPER_CAMERA)/maxWidth;
    difference = 0.4 * percent;
    double power = 0.5 - difference;
    
    lSpeed=(power+(targetAngle*0.01));
    rSpeed=(power-(targetAngle*0.01));
    
    Robot::drivetrainSub.drive(lSpeed,rSpeed);
  } else {
	  if (TimeSinceInitialized() > 0.6){
      if (!noLongerSeesTarget){
        noLongerSeesTarget = true;
        Robot::drivetrainSub.enableBalancerPID();
      }
      else {
        Robot::drivetrainSub.driverDriveStraight(0.2);
      } 
    } 
    else {
        Robot::drivetrainSub.drive(0.2,0.2);
    }
  } 
}

// Make this return true when this Command no longer needs to run execute()
bool VisionScoringCmd::IsFinished() { 
  if(driveAllTheWay){
    if ((Robot::visionSub.getHorizontalWidth(BUMPER_CAMERA) > maxWidth) && (fabs(Robot::visionSub.getVisionTarget(BUMPER_CAMERA)) < 2)) {
      return true;
    }
    else if(TimeSinceInitialized() - timeSinceTargetSeen > 0.75){
      return true;
    }
    else if (TimeSinceInitialized() > 4){
      return true;
    }
  }else{  
    // Subtracting 20 since we are not going all the way to the scoring target
    if ((Robot::visionSub.getHorizontalWidth(BUMPER_CAMERA) > maxWidth - 20) && (fabs(Robot::visionSub.getVisionTarget(BUMPER_CAMERA)) < 2)) {
      return true;
    }else if (TimeSinceInitialized() > timeSinceTargetSeen){
      return true;
    }else if (TimeSinceInitialized() > 4){
      return true;
    }
  }
  return false;
}

// Called once after isFinished returns true
void VisionScoringCmd::End() {
  Robot::drivetrainSub.drive(0,0);
  Robot::visionSub.setBumperPipeline(DRIVER_MODE_NORMAL);
  Robot::drivetrainSub.disableBalancerPID();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void VisionScoringCmd::Interrupted() {
  End();
}
