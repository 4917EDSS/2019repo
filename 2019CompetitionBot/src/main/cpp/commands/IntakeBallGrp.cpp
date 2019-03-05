/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include "commands/SetIntakeArmAngleCmd.h"
#include "commands/FoldIntakeCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/SetManipulatorCmd.h"
#include "commands/IntakeBallFromRobotCmd.h"
#include "commands/IntakeBallGrp.h"

IntakeBallGrp::IntakeBallGrp() {
  AddSequential(new SetIntakeArmAngleCmd(INTAKE_CARGO_ANGLE));
  AddParallel(new FoldIntakeCmd(false));
  AddSequential(new SetElevatorToHeightCmd(ELEVATOR_CARGO_FLOOR_PICKUP_HEIGHT_MM));
  AddSequential(new SetManipulatorCmd(MANIPULATOR_CARGO_FLOOR_PICKUP_ANGLE));
  
  // Enable intake and manipulator wheels, wait for ball detection, disable wheels
  AddSequential(new IntakeBallFromRobotCmd());
    
  AddSequential(new SetManipulatorCmd(-90));
  AddSequential(new SetElevatorToHeightCmd(ELEVATOR_MIN_HEIGHT_MM));
  AddSequential(new SetManipulatorCmd(0));
  AddParallel(new FoldIntakeCmd(true));
  AddSequential(new SetIntakeArmAngleCmd(INTAKE_NEUTRAL_ANGLE));
}
