/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ChangeGearCmd.h"
#include "Robot.h"
bool highGear;

ChangeGearCmd::ChangeGearCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void ChangeGearCmd::Initialize() {
highGear = Robot::elevatorSub.isShifterHigh();
}

// Called repeatedly when this Command is scheduled to run
void ChangeGearCmd::Execute() {
  if(highGear){
    Robot::elevatorSub.setShifterHigh(false);
  } else {
    Robot::elevatorSub.setShifterHigh(true);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ChangeGearCmd::IsFinished() { 
   if(highGear != Robot::elevatorSub.isShifterHigh()){
      return true;
    } else {
      return false;
    }
  }

// Called once after isFinished returns true
void ChangeGearCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ChangeGearCmd::Interrupted() {
  End();
}
