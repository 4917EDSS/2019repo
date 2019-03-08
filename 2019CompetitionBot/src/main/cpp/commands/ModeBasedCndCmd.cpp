/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ModeBasedCndCmd.h"
#include "Robot.h"


ModeBasedCndCmd::ModeBasedCndCmd(Command *onTrue, Command *onFalse) : frc::ConditionalCommand(onTrue, onFalse) {}

// First parameter (onTrue) is hatch height.  Second parameter (onFalse) is cargo height
bool ModeBasedCndCmd::Condition() {
  return !Robot::inBallMode;
}

