#pragma once
#include <frc/commands/Command.h>

class ManipulatorInCmd : public frc::Command {
  private:
    double intakeTime;
  public:
    ManipulatorInCmd(double intakeTime);
    ManipulatorInCmd();
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};
