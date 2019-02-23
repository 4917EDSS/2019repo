/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

class frc4917Cmd : public virtual frc::Command {
 public:
  virtual void Initialize() override = 0;
  virtual void Execute() override = 0;
  virtual bool IsFinished() override = 0;
  virtual void End() override = 0;
  virtual void Interrupted() override = 0;
};
