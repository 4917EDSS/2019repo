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

class ElevatorSub : public frc::Subsystem {
 private:
  std::shared_ptr<frc::Encoder> elevatorMotorEnc;
  std::shared_ptr<rev::CANSparkMax> elevatorMotor1;
  std::shared_ptr<rev::CANSparkMax> elevatorMotor2;
  std::shared_ptr<frc::DigitalInput> lowerLimit;
  std::shared_ptr<frc::DigitalInput> upperLimit;
  std::shared_ptr<frc::Solenoid> hatchGripperSolenoid;
  std::shared_ptr<WPI_VictorSPX> manipulatorIntakeMotorLeft;
  std::shared_ptr<WPI_VictorSPX> manipulatorIntakeMotorRight;
  std::shared_ptr<frc::DigitalInput> intakeFromRobotLimit;
  std::shared_ptr<rev::CANSparkMax> manipulatorFlipperMotor;
  std::shared_ptr<frc::DigitalInput> manipulatorFlipperLimit;
  
  double targetDegrees;
  double targetHeight;

 public:
  ElevatorSub();
  void InitDefaultCommand() override;

  void setElevatorMotorSpeed(double speed);
  void setElevatorTargetHeight(double newTarget);
  double getElevatorEncoder();
  bool isElevatorDown();
  void setElevatorMotorRaw(double speed);

  void setManipulatorFlipperMotorSpeed(double speed);
  void setManipulatorTargetAngle(double newAngle);
  double getManipulatorEncoder();
  bool isManipulatorAtLimit();

  void setManipulatorWheelSpeed(double lspeed, double rspeed);
  bool isBallInManipulator();

  void expandHatchGripper();  
  void contractHatchGripper();

  void update();
  void executeStateMachine();
  bool isFinishedMove();
  void zeroEverything();
};
