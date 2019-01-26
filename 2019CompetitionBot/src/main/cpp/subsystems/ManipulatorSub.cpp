/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ManipulatorSub.h"

ManipulatorSub::ManipulatorSub() : Subsystem("ManipulatorSub") {}

void ManipulatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  hatchGripperSolenoid.reset(new frc::Solenoid(HATCH_GRIPPER_PCM_ID));
}

void ManipulatorSub::ExpandHatchGripper(){
  hatchGripperSolenoid->Set(true);
}

void ManipulatorSub::ContractHatchGripper(){
  hatchGripperSolenoid->Set(false);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
