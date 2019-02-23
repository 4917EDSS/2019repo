/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/*
Encoder Heights

  Loading Station
    Hatch = -5.140
    Ball = -20.071

  Rocket Ship
    Ball
      Low = -11.714
      Medium = -36.21
      High = -52.524
    Hatch
      Low = -5.140
      Medium = -25.499
      High = -49.47
  
  Cargo Ship
    Hatch = -5.140
    Ball = -24.92
*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <RobotMap.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

// Elevator heights
// TODO:  Update these values to real ones.
constexpr double ELEVATOR_MIN_HEIGHT_MM = 500;
constexpr double ELEVATOR_MAX_HEIGHT_MM = 2000;
constexpr double ELEVATOR_LOW_HATCH_HEIGHT_MM = 500;
constexpr double ELEVATOR_MEDIUM_HATCH_HEIGHT_MM = 500;
constexpr double ELEVATOR_HIGH_HATCH_HEIGHT_MM = 500;
constexpr double ELEVATOR_ROCKET_LOW_CARGO_HEIGHT_MM = 500;
constexpr double ELEVATOR_ROCKET_MEDIUM_CARGO_HEIGHT_MM = 500;
constexpr double ELEVATOR_ROCKET_HIGH_CARGO_HEIGHT_MM = 500;
constexpr double ELEVATOR_CARGO_SHIP_CARGO_HEIGHT_MM = 500;

// Elevator height control modes
constexpr int ELEVATOR_MODE_DISABLED = 0;
constexpr int ELEVATOR_MODE_AUTO = 1;
constexpr int ELEVATOR_MODE_MANUAL = 2;

class ElevatorSub : public frc::Subsystem {
 private:
  std::shared_ptr<frc::Encoder> elevatorMotorEnc;
  std::shared_ptr<rev::CANSparkMax> elevatorMotor1;
  std::shared_ptr<rev::CANSparkMax> elevatorMotor2;
  std::shared_ptr<frc::DigitalInput> lowerLimit;
  std::shared_ptr<frc::DigitalInput> upperLimit;
  std::shared_ptr<frc::Solenoid> shifterSolenoid;
  std::shared_ptr<frc::Solenoid> hatchGripperSolenoid;
  std::shared_ptr<WPI_VictorSPX> manipulatorIntakeMotorLeft;
  std::shared_ptr<WPI_VictorSPX> manipulatorIntakeMotorRight;
  std::shared_ptr<frc::DigitalInput> intakeFromRobotLimit;
  std::shared_ptr<rev::CANSparkMax> manipulatorFlipperMotor;
  std::shared_ptr<frc::DigitalInput> manipulatorFlipperLimit;
  
  double targetAngle;
  double targetHeight;

  // Elevator state machine variables and functions
  bool elevatorNewStateParameters;
  int elevatorNewControlMode;
  int elevatorNewMaxPower;
  double elevatorNewTargetHeightMm;
  int elevatorNewState;
  int elevatorControlMode;
  int elevatorMaxPower;
  double elevatorTargetHeightMm;
  int elevatorState;
  double elevatorLastPower;
  double elevatorBlockedHeightMm;

  void setElevatorMotorPower(double power);
  double getElevatorHeight();
  double getElevatorVelocity();
  double calcElevatorHoldPower(double currentHeightMm, double targetHeightMm);
  double calcElevatorMovePower(double currentHeightMm, double targetHeightMm, double maxElevatorPower);
  bool isElevatorBlocked(double currentHeightMm);

 public:
  ElevatorSub();
  void InitDefaultCommand() override;

  void updateElevatorStateMachine();
  void setElevatorHeight(int mode, double maxPower, double targetHeightMm);
  bool isElevatorAtTarget();



  void setElevatorMotorSpeed(double speed);
  void setElevatorTargetHeight(double newTarget);
  double getElevatorEncoder();
  bool isElevatorDown();
  void setElevatorMotorRaw(double speed);

  void setShifterHigh(bool highGear);
  bool isShifterHigh();

  void setManipulatorFlipperMotorSpeed(double speed);
  void setManipulatorTargetAngle(double newAngle);
  double getManipulatorEncoder();
  bool isManipulatorAtLimit();
  void holdManipulatorFlipper(double position);
  double getManipulatorAngle();

  void setManipulatorWheelSpeed(double lspeed, double rspeed);
  bool isBallInManipulator();

  void expandHatchGripper();  
  void contractHatchGripper();
  bool isGripperExpanded();

  void update();
  bool isFinishedMove();
  void zeroEverything();
};
