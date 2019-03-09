/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ZeroAllSystemsGrp.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "subsystems/ElevatorSub.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/FoldIntakeCmd.h"
#include "commands/SetIntakeArmAngleCmd.h"






ZeroAllSystemsGrp::ZeroAllSystemsGrp() {
 AddSequential(new SetElevatorToHeightCmd(ELEVATOR_MID_SAFE_HEIGHT));
 AddParallel(new SetManipulatorAngleCmd(0.0));
 AddSequential(new FoldIntakeCmd(true));
 AddParallel(new SetIntakeArmAngleCmd(false, 0.0));
 AddSequential(new SetElevatorToHeightCmd(ELEVATOR_MIN_HEIGHT_MM));
}
