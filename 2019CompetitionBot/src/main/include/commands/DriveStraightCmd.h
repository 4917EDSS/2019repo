#pragma once
#include "frc/commands/Command.h"
class DriveStraightCmd : public frc::Command {
 public:
  DriveStraightCmd(double distance);
  void Initialize();
  void Execute();
  bool IsFinished();
  void End();
  void Interrupt();
 private:
  double distance;
};