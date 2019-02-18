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
#include "commands/SetElevatorToHeightCmd.h"

IntakeBallGrp::IntakeBallGrp() {

  AddParallel(new FoldIntakeCmd(true));
  AddSequential(new SetIntakeArmAngleCmd(false,90));
  AddSequential(new SetElevatorToHeightCmd(0.0));
  AddSequential(new IntakeBallFromRobotCmd());
}
