/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/HighVisionHatchGrp.h"
#include "commands/VisionScoringCmd.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/DriveStraightCmd.h"
#include "commands/CargoModeGrp.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "subsystems/ElevatorSub.h"
#include "commands/SetManipulatorIntakePowerCmd.h"
#include "commands/HatchGripperContractCmd.h"
#include "commands/ManipulatorInCmd.h"
#include "commands/HatchVisionCmd.h"

HighVisionHatchGrp::HighVisionHatchGrp() {

  AddSequential(new VisionScoringCmd(true));
  AddSequential(new SetManipulatorIntakePowerCmd(1.0));
  AddSequential(new frc::WaitCommand(0.15));
  AddSequential(new HatchGripperContractCmd());
  AddSequential(new ManipulatorInCmd(0.3));

}
