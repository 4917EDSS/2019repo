/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/CargoHighHeightGrp.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "subsystems/ElevatorSub.h"

CargoHighHeightGrp::CargoHighHeightGrp() {
  AddParallel(new SetManipulatorAngleCmd(70));
  AddSequential(new SetElevatorToHeightCmd(ELEVATOR_ROCKET_HIGH_CARGO_HEIGHT_MM));

}
