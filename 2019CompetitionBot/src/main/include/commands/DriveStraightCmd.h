#pragma once
#include "frc/commands/Command.h"
#include "commands/frc4917Cmd.h"

class DriveStraightCmd : public frc4917Cmd {
 public:
  DriveStraightCmd(double distance);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
 private:
  double distance;
};
