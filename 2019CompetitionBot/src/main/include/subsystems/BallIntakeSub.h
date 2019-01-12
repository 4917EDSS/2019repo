/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <RobotMap.h>


class BallIntakeSub : public frc::Subsystem {
 private:
  std::shared_ptr<ctre::phoenix::motorcontrol::can::VictorSPX> BallIntakeMotor;


  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  BallIntakeSub();
  void InitDefaultCommand() override;
  void SetIntakeMotor(double speed);
};
