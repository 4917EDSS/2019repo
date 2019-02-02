/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "frc/WPIlib.h"
#include "RobotMap.h"
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

class ManipulatorSub : public frc::Subsystem {
 private:
  std::shared_ptr<frc::Solenoid> hatchGripperSolenoid;
  std::shared_ptr<WPI_VictorSPX> manipulatorIntakeMotorLeft;
  std::shared_ptr<WPI_VictorSPX> manipulatorIntakeMotorRight;
  std::shared_ptr<frc::DigitalInput> intakeFromRobotLimit;
  std::shared_ptr<rev::CANSparkMax> manipulatorFlipperMotor;
  std::shared_ptr<frc::DigitalInput> manipulatorFlipperLimit;
  
  int targetDegrees;
  int currentState;
  int currentDegrees;
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  ManipulatorSub();
  void InitDefaultCommand() override;
  void ExpandHatchGripper();
  void ContractHatchGripper();
  void IntakeBall(double speed);
  bool isBallInManipulator();
  void flipManipulator(bool goForward);
  bool isManipulatorFlipped();
  void executeStateMachine();
};
