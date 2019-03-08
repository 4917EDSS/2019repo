/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/CargoModeGrp.h"
#include "commands/ManipulatorInCmd.h"
#include "commands/HatchGripperContractCmd.h"
#include "commands/ManipulatorOutCmd.h"
#include "Robot.h"

CargoModeGrp::CargoModeGrp() {
  // This command gets ready for cargo pickup but is also the hatch-expel command.

  // When we have a hatch, take the pressure off the gripper by expeling a bit before the contraction.
  AddSequential(new ManipulatorOutCmd(0.1));
  AddSequential(new HatchGripperContractCmd());
  AddSequential(new ManipulatorInCmd(1.0));
}
