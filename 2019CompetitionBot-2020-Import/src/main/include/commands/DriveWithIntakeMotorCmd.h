#pragma once
#include <frc/commands/Command.h>

class DriveWithIntakeMotorCmd : public frc::Command {
  public:
    DriveWithIntakeMotorCmd();
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End() override;
    void Interrupted() override;
};
