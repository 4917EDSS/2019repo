#pragma once
#include <frc/commands/Command.h>
#include "commands/frc4917Cmd.h"

class ManipulatorOutCmd : public frc4917Cmd
{
  public:
    ManipulatorOutCmd();
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};
