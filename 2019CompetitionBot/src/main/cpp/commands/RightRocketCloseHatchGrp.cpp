/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/RightRocketCloseHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetIntakeArmAngleCmd.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/SetElevatorToHeightCmd.h"


RightRocketCloseHatchGrp::RightRocketCloseHatchGrp() {

  AddSequential(new ExpandHatchGripperGrp());
  AddParallel(new SetManipulatorAngleCmd(90));
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM));
  AddSequential(new SilkyMotionCmd(std::vector<double> {2800,1500}, std::vector<double> {48,-20}));
}
