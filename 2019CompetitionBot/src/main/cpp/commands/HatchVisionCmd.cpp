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

constexpr double JOYSTICK_DEADBAND = 0.01;
HatchVisionCmd::HatchVisionCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::drivetrainSub);
}

// Called just before this Command runs the first time
void HatchVisionCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  noLongerSeesTarget = false;
  if (Robot::manipulatorSub.getFlipperAngle() < 0) {
    Robot::visionSub.setManipulatorPipeline(VISION_MODE_NORMAL); 
  }
  else{
    Robot::visionSub.setManipulatorPipeline(VISION_MODE_FLIPPED);
  }
}

// Called repeatedly when this Command is scheduled to run
void HatchVisionCmd::Execute() {
  
  std::shared_ptr<frc::Joystick> driverJoystick = Robot::oi.getDriverController();

  double targetAngle=Robot::visionSub.getVisionTarget(MANIPULATOR_CAMERA);
  double verticalOffset = Robot::visionSub.getVerticalOffset(MANIPULATOR_CAMERA);
  
  if(fabs(driverJoystick->GetX()) > JOYSTICK_DEADBAND){
    targetAngle += (driverJoystick->GetX()*20.0);
  }

  if (Robot::visionSub.isTargetVisible(MANIPULATOR_CAMERA) && (verticalOffset < 20.00) && !noLongerSeesTarget) {
    double lSpeed=(0);
    double rSpeed=(0);

    if (Robot::manipulatorSub.getFlipperAngle() > 0) {
      lSpeed=(0.2+(targetAngle*0.015));
      rSpeed=(0.2-(targetAngle*0.015));
    } else {
      lSpeed=(-0.2+(targetAngle*0.015));
      rSpeed=(-0.2-(targetAngle*0.015));
    }
    Robot::drivetrainSub.drive(lSpeed,rSpeed);
  } else {
	  if (TimeSinceInitialized() > 0.3){
      if (!noLongerSeesTarget){
        noLongerSeesTarget = true;
        Robot::drivetrainSub.enableBalancerPID();
      }
      if (Robot::manipulatorSub.getFlipperAngle() > 0){
        Robot::drivetrainSub.driverDriveStraight(0.3);
      }
      else {
        Robot::drivetrainSub.driverDriveStraight(-0.3);
      } 
    } 
    else {
      if (Robot::manipulatorSub.getFlipperAngle() > 0){
        Robot::drivetrainSub.drive(0.3,0.3);
      }
      else {
        Robot::drivetrainSub.drive(-0.3,-0.3);
      }
      
    }
  } 
}

// Make this return true when this Command no longer needs to run execute()
bool HatchVisionCmd::IsFinished() { 
  if (Robot::visionSub.getHorizontalWidth(MANIPULATOR_CAMERA) > 250) {
    return true;
  }
  else {
    return false; 
  }
}

// Called once after isFinished returns true
void HatchVisionCmd::End() {
  Robot::drivetrainSub.drive(0,0);
  Robot::visionSub.setManipulatorPipeline(DRIVER_MODE_NORMAL);
  Robot::drivetrainSub.disableBalancerPID();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void HatchVisionCmd::Interrupted() {
  End();
}
