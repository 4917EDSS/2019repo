#pragma once
#include <frc/commands/Command.h>
#include "commands/frc4917Cmd.h"

class ManipulatorInCmd : public frc4917Cmd {
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
