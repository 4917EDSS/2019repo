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

// Elevator heights
constexpr double ELEVATOR_MIN_HEIGHT_MM = 445;
constexpr double ELEVATOR_MAX_HEIGHT_MM = 1952.8;
constexpr double ELEVATOR_LOW_HATCH_HEIGHT_MM = 482.6;
constexpr double ELEVATOR_MEDIUM_HATCH_HEIGHT_MM = 1193.8;
constexpr double ELEVATOR_HIGH_HATCH_HEIGHT_MM = 1905;
constexpr double ELEVATOR_ROCKET_LOW_CARGO_HEIGHT_MM = 698.5;
constexpr double ELEVATOR_ROCKET_MEDIUM_CARGO_HEIGHT_MM = 1409.7;
constexpr double ELEVATOR_ROCKET_HIGH_CARGO_HEIGHT_MM = 1942.8;
constexpr double ELEVATOR_CARGO_SHIP_CARGO_HEIGHT_MM = 1168.4;

// Elevator height control modes
constexpr int ELEVATOR_MODE_DISABLED = 0;
constexpr int ELEVATOR_MODE_AUTO = 1;
constexpr int ELEVATOR_MODE_MANUAL = 2;

class ElevatorSub : public frc::Subsystem {
 private:
  std::shared_ptr<rev::CANSparkMax> elevatorMotor1;
  std::shared_ptr<rev::CANSparkMax> elevatorMotor2;
  std::shared_ptr<frc::Encoder> elevatorMotorEnc;
  std::shared_ptr<frc::DigitalInput> lowerLimit;
  std::shared_ptr<frc::DigitalInput> upperLimit;
  std::shared_ptr<frc::Solenoid> shifterSolenoid;

  struct SparkShuffleboardEntrySet nteSparksTwo[2];
  nt::NetworkTableEntry nteHatchGripperSolenoid;
  nt::NetworkTableEntry nteIntakeFromRobotLimit;
  nt::NetworkTableEntry nteShifterSolenoid;

  // Elevator state machine variables and functions
  bool elevatorNewStateParameters;
  int elevatorNewControlMode;
  double elevatorNewMaxPower;
  double elevatorNewTargetHeightMm;
  int elevatorNewState;
  int elevatorControlMode;
  double elevatorMaxPower;
  double elevatorTargetHeightMm;
  int elevatorState;
  double elevatorLastPower;
  double elevatorBlockedHeightMm;

  bool isElevatorBlocked(double currentHeightMm, double targetHeightMm);
  double calcElevatorHoldPower(double currentHeightMm, double targetHeightMm);
  double calcElevatorMovePower(double currentHeightMm, double targetHeightMm, double maxElevatorPower);
  
 public:
  ElevatorSub();
  void InitDefaultCommand() override;
  void updateShuffleBoard();
  void setElevatorMotorPower(double power);
  double getElevatorHeight();
  double getElevatorVelocity();
  bool isElevatorDown();
  void setShifterHigh(bool highGear);
  bool isShifterHigh();
  void setElevatorHeight(int mode, double maxPower, double targetHeightMm);
  bool isElevatorAtTarget();

  void updateElevatorStateMachine();
};
