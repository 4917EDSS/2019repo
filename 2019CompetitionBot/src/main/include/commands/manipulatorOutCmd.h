#pragma once
#include <frc/commands/Command.h>

class manipulatorOutCmd : public frc::Command
{
  public:
    manipulatorOutCmd();
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};