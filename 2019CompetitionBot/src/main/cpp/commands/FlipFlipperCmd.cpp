/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/FlipFlipperCmd.h"
#include "Robot.h"
#include "subsystems/BallIntakeSub.h"
#include <iostream>

FlipFlipperCmd::FlipFlipperCmd(enum FlipperDirection flipperDirection) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);

  currentFlipperDirection = flipperDirection;
  flipperPositionOut = false;
}

// Called just before this Command runs the first time
void FlipFlipperCmd::Initialize() {
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
      std::cout << "You goofed. You need to add a case in the FlipFlipperCmd." << std::endl;
  }

  Robot::ballIntakeSub.setFlipperPosition(flipperPositionOut);
}

// Called repeatedly when this Command is scheduled to run
void FlipFlipperCmd::Execute() {
  //Might need PID or sensor here to slow down flipper
}

// Make this return true when this Command no longer needs to run execute()
bool FlipFlipperCmd::IsFinished() { 
  //Do we wait for sensor or encoder, or do we just assume that it's done?
  return true; 
}

// Called once after isFinished returns true
void FlipFlipperCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void FlipFlipperCmd::Interrupted() {}