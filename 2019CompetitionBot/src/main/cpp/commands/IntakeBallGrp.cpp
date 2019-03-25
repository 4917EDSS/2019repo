/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "subsystems/BallIntakeSub.h"
#include "commands/SetIntakeArmAngleCmd.h"
#include "commands/FoldIntakeCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/IntakeBallFromRobotCmd.h"
#include "commands/IntakeBallGrp.h"
#include "commands/FoldIntakeImmediateCmd.h"
#include "commands/SetManipulatorIntakePowerCmd.h"


IntakeBallGrp::IntakeBallGrp() {
  AddSequential(new FoldIntakeCmd(false));
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_CARGO_FLOOR_PICKUP_HEIGHT_MM)); //510
  AddSequential(new SetIntakeArmAngleCmd(false, INTAKE_CARGO_ANGLE));
  AddParallel(new SetManipulatorIntakePowerCmd(1.0));
  AddSequential(new SetManipulatorAngleCmd(MANIPULATOR_CARGO_FLOOR_PICKUP_ANGLE));
  
  // Enable intake and manipulator wheels, wait for ball detection, disable wheels
  AddSequential(new IntakeBallFromRobotCmd());
  
  AddSequential(new SetManipulatorAngleCmd(0));
  AddSequential(new FoldIntakeImmediateCmd(true));
  AddSequential(new SetIntakeArmAngleCmd(false, INTAKE_NEUTRAL_ANGLE));
}
