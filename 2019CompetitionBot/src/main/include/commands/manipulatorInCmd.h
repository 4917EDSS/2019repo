#pragma once
#include <frc/commands/Command.h>

class manipulatorInCmd : public frc::Command
{
  public:
    manipulatorInCmd();
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};