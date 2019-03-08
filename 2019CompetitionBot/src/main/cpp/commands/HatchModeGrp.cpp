/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/HatchModeGrp.h"
#include "commands/ManipulatorOutCmd.h"
#include "commands/HatchGripperContractCmd.h"
#include "Robot.h"

HatchModeGrp::HatchModeGrp() {
  // This command expels any loaded cargo and prepares for hatch pick-up
  AddSequential(new HatchGripperContractCmd());
  AddSequential(new ManipulatorOutCmd(1.5));
}
