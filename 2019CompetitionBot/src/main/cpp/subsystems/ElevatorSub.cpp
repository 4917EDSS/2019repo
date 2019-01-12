/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "subsystems/ElevatorSub.h"
#include <ctre/Phoenix.h>

ElevatorSub::ElevatorSub() : Subsystem("ExampleSubsystem") {
elevatorMotor.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(ELEVATOR_MOTOR_CAN_ID));
}

void ElevatorSub::SetElevatorMotor(double speed){
elevatorMotor->Set(ControlMode::PercentOutput, speed);
}



void ElevatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
   
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
