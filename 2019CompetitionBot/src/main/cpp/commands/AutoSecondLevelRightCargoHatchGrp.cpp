/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoSecondLevelRightCargoHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/DriveStraightCmd.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/SetElevatorToHeightCmd.h"


AutoSecondLevelRightCargoHatchGrp::AutoSecondLevelRightCargoHatchGrp() {
  AddSequential(new ExpandHatchGripperGrp());
  AddParallel(new SetManipulatorAngleCmd(90));
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM));
  AddSequential(new SilkyMotionCmd(std::vector<double> {550, -550, 3850, 2000, 500}, std::vector<double> {0, 0, 35, -125, 0}));

}
