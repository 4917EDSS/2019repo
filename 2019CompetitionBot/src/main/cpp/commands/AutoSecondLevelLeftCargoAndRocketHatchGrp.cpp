/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoSecondLevelLeftCargoAndRocketHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/LowVisionScoreGrp.h"
#include "commands/VisionHatchPickupGrp.h"

AutoSecondLevelLeftCargoAndRocketHatchGrp::AutoSecondLevelLeftCargoAndRocketHatchGrp() {
  AddSequential(new ExpandHatchGripperGrp());
  AddParallel(new SetManipulatorAngleCmd(90.0));
  AddSequential(new SilkyMotionCmd(std::vector<double> {-1670, -3400, -900, 500}, std::vector<double> {0, -30, -60, 0})); 
}
