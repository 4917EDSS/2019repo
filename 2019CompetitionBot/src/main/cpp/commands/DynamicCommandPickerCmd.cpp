/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DynamicCommandPickerCmd.h"
#include "Robot.h"

DynamicCommandPickerCmd::DynamicCommandPickerCmd(frc::Command* ballMode, frc::Command* hatchMode) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  ballModeCmd = ballMode;
  hatchModeCmd = hatchMode;
}

// Called just before this Command runs the first time
void DynamicCommandPickerCmd::Initialize() {
  if (Robot::inBallMode) {
    //ballModeCmd->Initialize();
  } else {
    //hatchModeCmd->Initialize();
  }
}

// Called repeatedly when this Command is scheduled to run
void DynamicCommandPickerCmd::Execute() {
  if (Robot::inBallMode) {
    //ballModeCmd->Execute();
  } else {
    //hatchModeCmd->Execute();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool DynamicCommandPickerCmd::IsFinished() { 
  if (Robot::inBallMode) {
    //return ballModeCmd->IsFinished();
  } else {
    //return hatchModeCmd->IsFinished();
  }
}

// Called once after isFinished returns true
void DynamicCommandPickerCmd::End() {
  if (Robot::inBallMode) {
    //ballModeCmd->End();
  } else {
    //hatchModeCmd->End();
  }
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DynamicCommandPickerCmd::Interrupted() {
  if (Robot::inBallMode) {
    //ballModeCmd->Interrupted();
  } else {
    //hatchModeCmd->Interrupted();
  }
}
