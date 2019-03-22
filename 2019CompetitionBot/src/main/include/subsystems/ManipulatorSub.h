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
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include "components/MotorBalancer.h"
#include "SparkShuffleboardEntrySet.h"

constexpr double MANIPULATOR_MAX_ANGLE = 90;
constexpr double MANIPULATOR_MIN_ANGLE = -109;

constexpr double MANIPULATOR_CARGO_FLOOR_PICKUP_ANGLE = -109;

// Flipper angle control modes
constexpr int FLIPPER_MODE_DISABLED = 0;
constexpr int FLIPPER_MODE_AUTO = 1;
constexpr int FLIPPER_MODE_MANUAL = 2;

class ManipulatorSub : public frc::Subsystem {
 private:
  std::shared_ptr<rev::CANSparkMax> flipperMotor;
  std::shared_ptr<frc::DigitalInput> flipperLimit;
  std::shared_ptr<WPI_VictorSPX> intakeMotorLeft;
  std::shared_ptr<WPI_VictorSPX> intakeMotorRight;
  std::shared_ptr<frc::Solenoid> hatchGripperSolenoid;
  std::shared_ptr<frc::DigitalInput> intakeFromRobotLimit;
  
  struct SparkShuffleboardEntrySet nteFlipperMotor;
  nt::NetworkTableEntry nteFlipperLimit;
  nt::NetworkTableEntry nteIntakeMotorLeft;
  nt::NetworkTableEntry nteIntakeMotorRight;
  nt::NetworkTableEntry nteHatchGripperSolenoid;
  nt::NetworkTableEntry nteIntakeFromRobotLimit;

    // Flipper state machine variables and functions
  bool flipperNewStateParameters;
  int flipperNewControlMode;
  double flipperNewMaxPower;
  double flipperNewTargetAngle;
  int flipperNewState;
  int flipperControlMode;
  double flipperMaxPower;
  double flipperTargetAngle;
  int flipperState;
  double flipperLastPower;
  double flipperBlockedAngle;

  bool isFlipperBlocked(double currentAngle, double targetAngle);
  double calcFlipperHoldPower(double currentAngle, double targetAngle);
  double calcFlipperMovePower(double currentAngle, double targetAngle, double maxPower);

 public:
  ManipulatorSub();
  void InitDefaultCommand() override;
  void updateShuffleBoard();
  void setFlipperPower(double power);
  double getFlipperTargetAngle();
  double getFlipperAngle();
  double getFlipperVelocity();
  bool isFlipperAtLimit();
  void setIntakePower(double power);
  double getIntakePower();
  bool isBallIn();
  void expandHatchGripper();  
  void contractHatchGripper();
  bool isGripperExpanded();
  void setFlipperAngle(int mode, double maxPower, double targetAngle);
  bool isFlipperAtTarget();
  void SetManipulatorEncoderZero();

  void updateFlipperStateMachine();
};
