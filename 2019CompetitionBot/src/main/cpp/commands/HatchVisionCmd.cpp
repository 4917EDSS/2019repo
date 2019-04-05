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
  
  if(fabs(driverJoystick->GetX()) > JOYSTICK_DEADBAND){
    targetAngle += (driverJoystick->GetX()*20.0);
  }

  if (Robot::visionSub.isTargetVisible(MANIPULATOR_CAMERA) ){
    double lSpeed=(0);
    double rSpeed=(0);

    if (Robot::manipulatorSub.getFlipperAngle() > 0) {
      lSpeed=(0.1+(targetAngle*0.015));
      rSpeed=(0.1-(targetAngle*0.015));
    } else {
      lSpeed=(-0.1+(targetAngle*0.015));
      rSpeed=(-0.1-(targetAngle*0.015));
    }
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
