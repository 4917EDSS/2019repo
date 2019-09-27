/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/YeetSub.h"
#include <RobotMap.h>
#include "commands/DriveWithJoystickCmd.h"

YeetSub::YeetSub() : Subsystem("YeetSub") {
  leftMotor1.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_1_CAN_ID, 
    rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor1.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_1_CAN_ID, 
    rev::CANSparkMaxLowLevel::MotorType::kBrushless));
}

void YeetSub::InitDefaultCommand() {
  SetDefaultCommand(new DriveWithJoystickCmd());
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void YeetSub::drive(double lPower, double rPower)
{
		leftMotor1->Set(lPower);
		rightMotor1->Set(rPower);
}