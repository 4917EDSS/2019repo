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
#include "SparkShuffleboardEntrySet.h"


class BallIntakeSub : public frc::Subsystem {
 private:
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX> ballIntakeMotor;
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX> flipperMotorOne;
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX> flipperMotorTwo;
  std::shared_ptr<frc::Solenoid> intakeFolderSolenoid;
  std::shared_ptr<frc::Encoder> intakeArmEnc;
  std::shared_ptr<frc::DigitalInput> ballIntakeArmLimit;
  double targetAngle;
  double currentSpeed;

  nt::NetworkTableEntry nteBallIntakeMotor;
  nt::NetworkTableEntry nteFlipperMotorOne;
  nt::NetworkTableEntry nteFlipperMotorTwo;
  nt::NetworkTableEntry nteIntakeFolderSolenoid;
  nt::NetworkTableEntry nteBallIntakeArmLimit;
  nt::NetworkTableEntry nteIntakeArmEncPosition;

 public:
  BallIntakeSub();
  void InitDefaultCommand() override;
  void ExtendRearIntakeSolenoids();
  void setIntakeWheelsMotorSpeed(double speed);
  double getIntakeWheelsMotorSpeed();
  void setFolderOut(bool flipOut);
  bool isFolderOut();
  double getIntakeArmEncoderAngle();
  bool getIntakeArmLimit();
  void setIn();
  void setOut();
  void setDown();
  void setArmTargetPosition(double angle);
  void setIntakeArmMotor(double speed, bool isClimbing);
  void update(bool isClimbing);
  void keepArmAtTarget(double speed, bool isClimbing);
  bool doneFlipping();
  void updateShuffleBoard();
};
