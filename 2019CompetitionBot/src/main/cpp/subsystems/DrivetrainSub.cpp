/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DrivetrainSub.h"

DrivetrainSub::DrivetrainSub() : Subsystem("ExampleSubsystem") {}

void DrivetrainSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void DrivetrainSub::drive(double lSpeed, double rSpeed){

  rightMotor1->Set(rSpeed);
  rightMotor2->Set(rSpeed);
  rightMotor3->Set(rSpeed);
  leftMotor1->Set(lSpeed);
  leftMotor2->Set(lSpeed);
  leftMotor3->Set(lSpeed);

}

// Put methods for controlling this subsystem
// here. Call these from Commands.
