/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/LeftRocketCloseHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetIntakeArmAngleCmd.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/SetManipulatorAngleCmd.h"

LeftRocketCloseHatchGrp::LeftRocketCloseHatchGrp() {
 
  AddSequential(new ExpandHatchGripperGrp());
  AddParallel(new SetManipulatorAngleCmd(90));
  AddSequential(new SilkyMotionCmd(std::vector<double> {3000,1600}, std::vector<double> {-45,17}));

}
