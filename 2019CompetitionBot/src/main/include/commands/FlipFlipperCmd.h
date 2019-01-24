/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

class FlipFlipperCmd : public frc::Command {
 public:
  enum FlipperDirection {out, in, toggle};

  FlipFlipperCmd(enum FlipperDirection flipperDirection);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

private:
  enum FlipperDirection currentFlipperDirection;
  bool flipperPositionOut;
};
