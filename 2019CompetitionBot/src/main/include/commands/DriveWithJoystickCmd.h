/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>

constexpr uint64_t AHRS_DELAY_TIME = 350000;
class DriveWithJoystickCmd : public frc::Command {
 public:
  DriveWithJoystickCmd();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
 private:
  double currentRotatePower;
  double currentDrivePower;
  int wasDrivingStraight;
  uint64_t timeSinceDrivingStraight;

};
