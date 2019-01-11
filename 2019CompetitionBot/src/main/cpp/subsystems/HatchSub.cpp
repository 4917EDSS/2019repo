/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/HatchSub.h"

HatchSub::HatchSub() : Subsystem("HatchSub") {
  hatchGripperSolenoid.reset(new frc::Solenoid(HATCH_GRIPPER_PCM_ID));
  hatchGripperSolenoid->Set(true);
}

void HatchSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void HatchSub::ExpandHatchGripper(){
  hatchGripperSolenoid->Set(true);
}
void HatchSub::ContractHatchGripper(){
  hatchGripperSolenoid->Set(false);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
