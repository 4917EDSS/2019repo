/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/ConditionalCommand.h>

class ModeBasedCndCmd : public frc::ConditionalCommand {
 public:
  ModeBasedCndCmd(Command *onTrue, Command *onFalse);
  bool Condition() override;
};
