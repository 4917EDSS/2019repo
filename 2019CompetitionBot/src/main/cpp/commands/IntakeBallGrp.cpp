/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeBallGrp.h"
#include "commands/IntakeBallFromRobotCmd.h"
#include "commands/SetIntakeArmAngleCmd.h"
#include "commands/FoldIntakeCmd.h"
#include "commands/SetElevatorandManipulatorCmd.h"
#include <iostream>
IntakeBallGrp::IntakeBallGrp() {
//false = pnematics pulled in
  AddParallel(new FoldIntakeCmd(false));
  //AddParallel(new FoldIntakeCmd(false));
  AddSequential(new SetIntakeArmAngleCmd(110));
  //std::cout << "intake ball group working";
  //AddSequential(new SetElevatorandManipulatorCmd(-90,0));
  //AddSequential(new IntakeBallFromRobotCmd());
}
