/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DynamicCommandPickerCmd.h"
#include "Robot.h"
#include "commands/frc4917Cmd.h"
#include "commands/frc4917Grp.h"

template <typename CBall, typename CHatch>
DynamicCommandPickerCmd<CBall,CHatch>::DynamicCommandPickerCmd(CBall* ballMode, CHatch* hatchMode) {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  ballModeCmd = ballMode;
  hatchModeCmd = hatchMode;
}

// Called just before this Command runs the first time
template <typename CBall, typename CHatch>
void DynamicCommandPickerCmd<CBall,CHatch>::Initialize() {
  inBallMode=Robot::inBallMode;
  if (inBallMode) {
    ballModeCmd->Initialize();
  } else {
    hatchModeCmd->Initialize();
  }
}

// Called repeatedly when this Command is scheduled to run
template <typename CBall, typename CHatch>
void DynamicCommandPickerCmd<CBall,CHatch>::Execute() {
  if (inBallMode) {
    ballModeCmd->Execute();
  } else {
    hatchModeCmd->Execute();
  }
}

// Make this return true when this Command no longer needs to run execute()
template <typename CBall, typename CHatch>
bool DynamicCommandPickerCmd<CBall,CHatch>::IsFinished() { 
  if (inBallMode) {
    return ballModeCmd->IsFinished();
  } else {
    return hatchModeCmd->IsFinished();
  }
}

// Called once after isFinished returns true
template <typename CBall, typename CHatch>
void DynamicCommandPickerCmd<CBall,CHatch>::End() {
  if (inBallMode) {
    ballModeCmd->End();
  } else {
    hatchModeCmd->End();
  }
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
template <typename CBall, typename CHatch>
void DynamicCommandPickerCmd<CBall,CHatch>::Interrupted() {
  if (inBallMode) {
    ballModeCmd->Interrupted();
  } else {
    hatchModeCmd->Interrupted();
  }
}

template class DynamicCommandPickerCmd<frc4917Grp, frc4917Grp>;
template class DynamicCommandPickerCmd<frc4917Cmd, frc4917Grp>;
template class DynamicCommandPickerCmd<frc4917Grp, frc4917Cmd>;
template class DynamicCommandPickerCmd<frc4917Cmd, frc4917Cmd>;