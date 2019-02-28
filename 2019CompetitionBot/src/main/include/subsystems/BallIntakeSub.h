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
constexpr int INTAKE_ARM_MODE_DISABLED = 0;
constexpr int INTAKE_ARM_MODE_AUTO = 1;
constexpr int INTAKE_ARM_MODE_MANUAL = 2;

class BallIntakeSub : public frc::Subsystem {
 private:
  std::shared_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX> ballIntakeMotor;
  std::shared_ptr<rev::CANSparkMax> flipperMotorOne;
  std::shared_ptr<frc::Solenoid> intakeFolderSolenoid;
  std::shared_ptr<frc::Encoder> intakeArmEnc;
  std::shared_ptr<frc::DigitalInput> ballIntakeArmLimit;
  double currentSpeed;

  nt::NetworkTableEntry nteBallIntakeMotor;
  nt::NetworkTableEntry nteFlipperMotorOne;
  nt::NetworkTableEntry nteFlipperMotorTwo;
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
  void setIntakeArmPower(double speed, bool isClimbing);
  bool getIntakeArmSpeed();
  double calcIntakeArmMovePower(double currentAngle, double targetAngle, double maxPower);
  void setArmTargetPosition(int mode, double maxPower, double targetAngle);
  void keepArmAtTarget(double speed, bool isClimbing);
  void updateIntakeArmStateMachine();
  double calcIntakeArmHoldPower(double currentAngle, double targetAngle);
  bool isIntakeArmBlocked(double currentAngle, double targetAngle);
  bool isIntakeArmAtTarget();
  void update(bool isClimbing, double targetAngle);
  bool doneFlipping();
  void updateShuffleBoard();
};
