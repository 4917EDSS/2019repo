/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DrivetrainSub.h"


DrivetrainSub::DrivetrainSub() : Subsystem("ExampleSubsystem") {
  //leftMotor1.reset(new rev::CANSparkMax(1, (rev::MotorType)1)); //second one is brushless type, put 0 for brushed motor
  //leftMotor2.reset(new rev::CANSparkMax(2, 1));
  //leftMotor3.reset(new rev::CANSparkMax(3, 1));
  leftMotor4.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(4));
  //rightMotor1.reset(new rev::CANSparkMax(5, 1));
  //rightMotor2.reset(new rev::CANSparkMax(6, 1));
  //rightMotor3.reset(new rev::CANSparkMax(7, 1));
}

void DrivetrainSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void DrivetrainSub::drive(double lSpeed, double rSpeed){

  // rightMotor1->Set(rSpeed);
  // rightMotor2->Set(rSpeed);
  // rightMotor3->Set(rSpeed);
  // leftMotor1->Set(lSpeed);
  // leftMotor2->Set(lSpeed);
  // leftMotor3->Set(lSpeed);
  leftMotor4->Set(ControlMode::PercentOutput, lSpeed);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
