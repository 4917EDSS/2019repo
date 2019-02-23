/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "commands/frc4917Cmd.h"

class SetElevatorandManipulatorCmd : public frc4917Cmd {
 public:
  SetElevatorandManipulatorCmd(double targetAngle, double targetHeight);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

  private:
  double targetAngle;
  double targetHeight;
};
