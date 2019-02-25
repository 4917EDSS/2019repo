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

  double targetAngle;

 public:
  ManipulatorSub();
  void InitDefaultCommand() override;
  void updateShuffleBoard();
  void setFlipperPower(double power);
  double getFlipperAngle();
  double getFlipperVelocity();
  bool isFlipperAtLimit();
  void setIntakePower(double power);
  bool isBallIn();
  void expandHatchGripper();  
  void contractHatchGripper();
  bool isGripperExpanded();


  void setManipulatorFlipperMotorSpeed(double speed);
  void setManipulatorTargetAngle(double newAngle);
  bool isManipulatorAtLimit();
  void holdManipulatorFlipper(double position);
  double getManipulatorAngle();

  bool isFinishedMove();
  void zeroEverything();
};
