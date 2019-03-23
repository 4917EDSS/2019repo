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

constexpr double CLIMB_RETRACT_LIMIT_THRESHOLD = 10;
constexpr double CLIMB_EXTEND_LIMIT_THRESHOLD = 570;

class ClimbSub : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
	std::shared_ptr<WPI_TalonSRX> climbMotor;

  nt::NetworkTableEntry ntePower;
  nt::NetworkTableEntry ntePosition;
  nt::NetworkTableEntry ntePitch;

  double prevPower;

 public:
  ClimbSub();
  void InitDefaultCommand() override;
  void updateShuffleBoard();
  void SetClimbMotorPower(double power);
  double getClimbPosition();
  void SetClimbEncoderZero();
};
