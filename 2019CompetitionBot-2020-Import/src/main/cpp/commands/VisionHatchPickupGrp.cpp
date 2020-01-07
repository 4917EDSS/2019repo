/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/VisionHatchPickupGrp.h"
#include "commands/HatchVisionCmd.h"
#include "commands/DriveStraightCmd.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/FlipManipulatorGrp.h"
#include "commands/ModeBasedCndCmd.h"
#include "frc/commands/InstantCommand.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "subsystems/ElevatorSub.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/SideBaseCmd.h"

VisionHatchPickupGrp::VisionHatchPickupGrp() {

AddParallel(new SideBaseCmd(new SetElevatorToHeightCmd(ELEVATOR_MIN_SAFE_HEIGHT), new frc::InstantCommand()));
AddSequential(new SetManipulatorAngleCmd(-90, true)); // Second  parameter removes the "requires Manipulator Sub" 
AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM + 100.0));
AddSequential(new HatchVisionCmd());
AddParallel(new DriveStraightCmd(0.5, 0.1));
AddSequential(new ExpandHatchGripperGrp());
//AddParallel(new DriveStraightCmd(0.5, -1.0));
//AddSequential(new FlipManipulatorGrp());

}
