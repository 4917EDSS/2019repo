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
  std::shared_ptr<ctre::phoenix::motorcontrol::can::VictorSPX> ballIntakeMotor;
  std::shared_ptr<ctre::phoenix::motorcontrol::can::VictorSPX> flipperMotorOne;
  std::shared_ptr<ctre::phoenix::motorcontrol::can::VictorSPX> flipperMotorTwo;
  std::shared_ptr<frc::Solenoid> intakeFolderSolenoid;
  std::shared_ptr<frc::Encoder> intakeArmEnc;
  double targetAngle;
  double speed;


 public:
  BallIntakeSub();
  void InitDefaultCommand() override;
  void ExtendRearIntakeSolenoids();
  void setIntakeMotor(double speed);
  void setFlipperOut(bool flipOut);
  double getIntakeArmEncoderAngle();
  void setIn();
  void setOut();
  void setDown();
  void setArmTargetPosition(double angle);
  void setIntakeArmMotor(double speed);
  void update(bool isClimbing);
  bool doneFlipping();
};
