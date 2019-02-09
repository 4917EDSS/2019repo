/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeBallGrp.h"
#include "commands/IntakeBallUntilLimitCmd.h"
#include "commands/IntakeBallFromRobotCmd.h"

IntakeBallGrp::IntakeBallGrp() {

  AddParallel(new IntakeBallUntilLimitCmd());
  //AddParallel(new SetIntakeAngleCmd());
  AddParallel(new IntakeBallFromRobotCmd());
}
