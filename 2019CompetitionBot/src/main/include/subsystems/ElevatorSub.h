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
  // This is where we will set the default values for the go to height ability
  // Don't know why there are two functions, this one is not needed I (Jimmy) think???
  //void SetElevatorMotor(double speed);
  void InitDefaultCommand() override;
  void update();
  double getElevatorEncoder();
  void setElevatorMotorSpeed(double speed);
  void setElevatorTargetHeight(double newTarget);
  void setManipulatorTargetAngle(double newAngle);
  bool isFinishedMove();
  bool isElevatorDown();
  void setElevatorMotorRaw(double speed);
  
  void ExpandHatchGripper();  
  void ContractHatchGripper();
  double getManipulatorEncoder();
  void setManipulatorWheelSpeed(double lspeed, double rspeed);
  bool isBallInManipulator();
  void executeStateMachine();
  bool isManipulatorAtLimit();
  void zeroEverything();
  void setManipulatorFlipperMotorSpeed(double speed);
};
