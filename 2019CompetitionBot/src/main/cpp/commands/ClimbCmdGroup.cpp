/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "Robot.h"
#include "commands/ClimbCmdGroup.h"
#include "commands/FlipFlipperCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/ChangeGearCmd.h"
#include "commands/DriveStraightCmd.h"

constexpr double RAMP_DRIVE_DISTANCE = 3.5;

ClimbCmdGroup::ClimbCmdGroup() {
  // Add Commands here:
  // e.g. AddSequential(new Command1());
  //      AddSequential(new Command2());
  // these will run in order.

  // To run multiple commands at the same time,
  // use AddParallel()
  // e.g. AddParallel(new Command1());
  //      AddSequential(new Command2());
  // Command1 and Command2 will run in parallel.

  // A command group will require all of the subsystems that each member
  // would require.
  // e.g. if Command1 requires chassis, and Command2 requires arm,
  // a CommandGroup containing them would require both the chassis and the
  // arm.
  AddParallel(new SetElevatorToHeightCmd(UP_CLIMB_POSITION));
  //lock foot
  AddSequential(new FlipFlipperCmd(FlipFlipperCmd::out));
  //
  AddSequential(new ChangeGearCmd());
  AddSequential(new DriveStraightCmd(RAMP_DRIVE_DISTANCE));

}
