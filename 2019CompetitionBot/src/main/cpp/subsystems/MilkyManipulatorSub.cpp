/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/MilkyManipulatorSub.h"
#include "Commands/MilkyManipulatorWithJoystickCmd.h"

//MilkyManipulatorSub::MilkyManipulatorSub() : Subsystem("MilkyManipulatorSub") {


}

void MilkyManipulatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  //SetDefaultCommand(new MilkyManipulatorWithJoystickCmd());
}

void MilkyManipulatorSub::milkyManipulator(double speed){
  //milkyManipulator(speed, speed);
}

void MilkyManipulatorSub::milkyManipulator(double leftSpeed, double rightSpeed) {
  //milkyManipulatorMotorLeft->Set(ControlMode::PercentOutput, -leftSpeed);
 // milkyManipulatorMotorRight->Set(ControlMode::PercentOutput, rightSpeed);

}


// Put methods for controlling this subsystem
// here. Call these from Commands.
