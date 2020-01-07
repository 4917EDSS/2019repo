#pragma once
#include <frc/commands/Command.h>

class ManipulatorOutCmd : public frc::Command
{
  private:
    double intakeTime;
  public:
    ManipulatorOutCmd(double intakeTime);
    ManipulatorOutCmd();
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};
