/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <Commands/Subsystem.h>
#include <WPILib.h>
//#include <rev/CANSparkMax.h>

class DrivetrainSub : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  std::shared_ptr <frc::Talon> leftMotor1;
  std::shared_ptr <frc::Talon> leftMotor2;
  std::shared_ptr <frc::Talon> leftMotor3;
  std::shared_ptr <frc::Talon> rightMotor1;
  std::shared_ptr <frc::Talon> rightMotor2;
  std::shared_ptr <frc::Talon> rightMotor3;

 public:
  DrivetrainSub();
  void InitDefaultCommand() override;
  void drive(double lSpeed, double rSpeed);

};
