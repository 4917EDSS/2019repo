/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "commands/frc4917Cmd.h"


class SetElevatorCmd : public frc4917Cmd {
 public:
  SetElevatorCmd(double targetHeight);
  void Execute() override;
  void Initialize() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

 private:
  double targetHeight;
};
