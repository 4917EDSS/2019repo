#pragma once
#include <frc/commands/Command.h>
#include "commands/frc4917Cmd.h"

class manipulatorInCmd : public frc4917Cmd {
  public:
    manipulatorInCmd();
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};
