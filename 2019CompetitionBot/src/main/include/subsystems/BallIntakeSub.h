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
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <RobotMap.h>
#include "SparkShuffleboardEntrySet.h"

constexpr double INTAKE_NEUTRAL_ANGLE = 0; // Only angle that works
constexpr double INTAKE_CARGO_ANGLE = 90;   // TODO:  Check and update

constexpr int INTAKE_ARM_MODE_DISABLED = 0;
constexpr int INTAKE_ARM_MODE_AUTO = 1;
constexpr int INTAKE_ARM_MODE_MANUAL = 2;

class BallIntakeSub : public frc::Subsystem {
 private:
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX> ballIntakeMotor;
  double intakeWheelPower;
  std::shared_ptr<rev::CANSparkMax> flipperMotor;
  std::shared_ptr<frc::Solenoid> intakeFolderSolenoid;
  std::shared_ptr<frc::DigitalInput> ballIntakeArmLimit;

  struct SparkShuffleboardEntrySet nteFlipperMotor;
  nt::NetworkTableEntry nteBallIntakeMotor;
  nt::NetworkTableEntry nteIntakeFolderSolenoid;
  nt::NetworkTableEntry nteBallIntakeArmLimit;
  nt::NetworkTableEntry nteIntakeArmEncPosition;

    // Flipper state machine variables and functions
  bool intakeArmNewStateParameters;
  int intakeArmNewControlMode;
  double intakeArmNewMaxPower;
  double intakeArmNewTargetAngle;
  int intakeArmNewState;
  int intakeArmControlMode;
  double intakeArmMaxPower;
  double intakeArmTargetAngle;
  int intakeArmState;
  double intakeArmLastPower;
  double intakeArmBlockedAngle;

  bool isIntakeArmBlocked(double currentAngle, double targetAngle);
  double calcIntakeArmHoldPower(double currentAngle, double targetAngle);
  double calcIntakeArmMovePower(double currentAngle, double targetAngle, double maxPower);

 public:
  BallIntakeSub();
  void InitDefaultCommand() override;
  void updateShuffleBoard();
  void setIntakeArmPower(double power);
  double getIntakeArmAngle();
  double getIntakeArmVelocity();
  bool isIntakeAtLimit();
  void unfoldIntakeArms();
  void foldIntakeArms();
  bool isIntakeUnfolded();
  void setIntakeWheelPower(double speed);
  double getIntakeWheelPower();
  void setIntakeArmAngle(int mode, double maxPower, double targetAngle);
  bool isIntakeArmAtTarget();
  void SetBallIntakeEncoderZero();

  void updateIntakeArmStateMachine();  
};
