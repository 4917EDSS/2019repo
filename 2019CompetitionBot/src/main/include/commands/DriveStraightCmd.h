#pragma once
#include "frc/commands/Command.h"

class DriveStraightCmd : public frc::Command {
 public:
  DriveStraightCmd(double distance, double power);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
 private:
  double time;
  double power;
  double timeSinceStarted;
};
