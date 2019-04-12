/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoVision2ndLevelLCargoHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/SetElevatorToHeightCmd.h"

AutoVision2ndLevelLCargoHatchGrp::AutoVision2ndLevelLCargoHatchGrp() {

  AddSequential(new ExpandHatchGripperGrp());
  AddParallel(new SetManipulatorAngleCmd(90));
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM));
  AddSequential(new SilkyMotionCmd(std::vector<double> {550, -200, 4000}, std::vector<double> {0, 0, -30}));
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_MEDIUM_HATCH_HEIGHT_MM));
  AddSequential(new SilkyMotionCmd(std::vector<double> { 2000, 500}, std::vector<double> {120, 0}));
  // Vision to close-to-ship
  // Lower elevator
  // Forward to ship
}
