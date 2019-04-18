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
VisionScoringCmd::VisionScoringCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::drivetrainSub);
}

// Called just before this Command runs the first time
void VisionScoringCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  noLongerSeesTarget = false;
  Robot::visionSub.setBumperPipeline(VISION_MODE_NORMAL); 
  timeSinceTargetSeen = 99999;
}

// Called repeatedly when this Command is scheduled to run
void VisionScoringCmd::Execute() {
  
  std::shared_ptr<frc::Joystick> driverJoystick = Robot::oi.getDriverController();

  double targetAngle=Robot::visionSub.getVisionTarget(BUMPER_CAMERA);
  double verticalOffset = Robot::visionSub.getVerticalOffset(BUMPER_CAMERA);
  
  if(fabs(driverJoystick->GetX()) > JOYSTICK_DEADBAND){
    targetAngle += (driverJoystick->GetX()*20.0);
  }

  if (Robot::visionSub.isTargetVisible(BUMPER_CAMERA) && (verticalOffset < 13.00) && !noLongerSeesTarget) {
    double lSpeed=(0);
    double rSpeed=(0);
    double percent;
    double difference;
    percent = Robot::visionSub.getHorizontalWidth(BUMPER_CAMERA)/250.0;
    difference = 0.35 * percent;
    double power = 0.6 - difference;
    
    lSpeed=(power+(targetAngle*0.017));
    rSpeed=(power-(targetAngle*0.017));
    
    Robot::drivetrainSub.drive(lSpeed,rSpeed);
  } else {
	  if (TimeSinceInitialized() > 0.6){
      if (!noLongerSeesTarget){
        noLongerSeesTarget = true;
        timeSinceTargetSeen = TimeSinceInitialized();
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
  if (Robot::visionSub.getHorizontalWidth(BUMPER_CAMERA) > 250) {
    return true;
  }
  else if(TimeSinceInitialized() - timeSinceTargetSeen > 0.25){
    return true;
  }
  else if (TimeSinceInitialized() > 4){
    return true;
  }
  else {
    return false; 
  }
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
