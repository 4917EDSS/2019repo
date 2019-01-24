/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

class MilkyManipulatorWithJoystickCmd : public frc::Command {
 public:
  MilkyManipulatorWithJoystickCmd();
  void Initialize();
  void Execute();
  bool IsFinished();
  void End();
  void Interrupted();
};
