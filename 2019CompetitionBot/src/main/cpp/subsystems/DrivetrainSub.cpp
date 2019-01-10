/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DrivetrainSub.h"


DrivetrainSub::DrivetrainSub() : Subsystem("ExampleSubsystem") {

  // Todo - use proper CAN ID defines
  leftMotor1.reset(new rev::CANSparkMax(1, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor2.reset(new rev::CANSparkMax(2, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor3.reset(new rev::CANSparkMax(3, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

  rightMotor1.reset(new rev::CANSparkMax(5, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor2.reset(new rev::CANSparkMax(6, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor3.reset(new rev::CANSparkMax(7, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

  // Todo - remove
  extraMotor.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(4));
}

void DrivetrainSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void DrivetrainSub::drive(double lSpeed, double rSpeed){

  leftMotor1->Set(lSpeed);
  leftMotor2->Set(-lSpeed);
  leftMotor3->Set(lSpeed);

  rightMotor1->Set(rSpeed);
  rightMotor2->Set(-rSpeed);
  rightMotor3->Set(rSpeed);
  
  // Todo - remove
  extraMotor->Set(ControlMode::PercentOutput, lSpeed);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
