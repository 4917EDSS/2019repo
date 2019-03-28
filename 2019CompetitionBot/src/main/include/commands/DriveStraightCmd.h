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
  double distance;
  double power;
};
