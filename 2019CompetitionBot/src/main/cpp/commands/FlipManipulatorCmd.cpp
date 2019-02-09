/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FlipManipulatorCmd.h"
#include "Robot.h"
#include "subsystems/BallIntakeSub.h"
#include <iostream>

FlipManipulatorCmd::FlipManipulatorCmd(FlipperDirection flipperDirection) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);

  currentFlipperDirection = flipperDirection;
  flipperPositionOut = false;
}

// Called just before this Command runs the first time
void FlipManipulatorCmd::Initialize() {
  switch (currentFlipperDirection) {
    case out: 
      flipperPositionOut = true;
      break;
    case in:
      flipperPositionOut = false;
      break;
    case toggle:
      flipperPositionOut = !flipperPositionOut;
      break;
    default:
      std::cout << "You goofed. You need to add a case in the FlipManipulatorCmd." << std::endl;
  }
 
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::ballIntakeSub.setFlipperOut(flipperPositionOut);
}

// Called repeatedly when this Command is scheduled to run
void FlipManipulatorCmd::Execute() {
  //Might need PID or sensor here to slow down flipper
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);
}

// Make this return true when this Command no longer needs to run execute()
bool FlipManipulatorCmd::IsFinished() { 
  //Do we wait for sensor or encoder, or do we just assume that it's done?
  return true; 
}

// Called once after isFinished returns true
void FlipManipulatorCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FlipManipulatorCmd::Interrupted() {}
