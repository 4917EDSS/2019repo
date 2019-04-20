/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoSecondLevelLeftCargoAndRocketHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/SilkyMotionCmd.h"

AutoSecondLevelLeftCargoAndRocketHatchGrp::AutoSecondLevelLeftCargoAndRocketHatchGrp() {
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

  AddSequential(new ExpandHatchGripperGrp());
  AddSequential(new SilkyMotionCmd(std::vector<double> {-1000, -4000, -1000}, std::vector<double> {0, -30, -60}));
  //***Add vision delivery and drop off here***
  AddSequential(new WaitCommand(15));
  AddSequential(new SilkyMotionCmd(std::vector<double> {-2000, -1000, -1500}, std::vector<double> {-60, 0, -30}));

}
