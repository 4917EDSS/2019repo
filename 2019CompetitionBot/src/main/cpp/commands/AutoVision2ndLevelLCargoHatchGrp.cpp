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
#include "commands/VisionScoringCmd.h"

AutoVision2ndLevelLCargoHatchGrp::AutoVision2ndLevelLCargoHatchGrp() {

  AddSequential(new ExpandHatchGripperGrp());
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM));
  AddParallel(new SetManipulatorAngleCmd(0));
  AddSequential(new SilkyMotionCmd(std::vector<double> {550, -550, 5000, 500}, std::vector<double> {0, 0, -35, 115}));

  // Vision to close-to-ship
  AddSequential(new VisionScoringCmd());
  AddSequential(new SilkyMotionCmd(std::vector<double> {-400}, std::vector<double> {0}));
  AddSequential(new SetManipulatorAngleCmd(90));
  AddSequential(new SilkyMotionCmd(std::vector<double> {450}, std::vector<double> {0}));
  // Forward to ship
}
