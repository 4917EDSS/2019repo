#pragma once
#include <frc/commands/Command.h>
#include "commands/frc4917Cmd.h"

class DriveWithIntakeMotorCmd : public frc4917Cmd {
  public:
    DriveWithIntakeMotorCmd();
    void Initialize() override;
  void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};
