/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc/WPIlib.h"
#include "RobotMap.h"

class HatchSub : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  std::shared_ptr<frc::Solenoid> hatchGripperSolenoid;
 public:
  HatchSub();
  void InitDefaultCommand() override;
  void ExpandHatchGripper();
  void ContractHatchGripper();
};
