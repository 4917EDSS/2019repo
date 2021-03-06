/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoVisionFirstLevelRightRocketTwoHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/VisionScoringCmd.h"
#include "commands/CargoModeGrp.h"
#include "commands/VisionHatchPickupGrp.h"
#include "commands/SetManipulatorIntakePowerCmd.h"
#include "commands/HatchGripperContractCmd.h"
#include "commands/ManipulatorInCmd.h"
#include "commands/DoNothingHatchCmd.h"
#include "commands/HatchGripperExpandCmd.h"

AutoVisionFirstLevelRightRocketTwoHatchGrp::AutoVisionFirstLevelRightRocketTwoHatchGrp() {
  AddParallel(new DoNothingHatchCmd());
  AddSequential(new HatchGripperExpandCmd());
  AddSequential(new SetManipulatorIntakePowerCmd(-0.25));
  AddParallel(new SetElevatorToHeightCmd((ELEVATOR_LOW_HATCH_HEIGHT_MM + ELEVATOR_MEDIUM_HATCH_HEIGHT_MM)/2));
  AddParallel(new SetManipulatorAngleCmd(90));
  AddSequential(new SilkyMotionCmd(std::vector<double> {500, 1700, 400}, std::vector<double> {0, 80, -52}));
  // Vision to close-to-ship
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_MEDIUM_HATCH_HEIGHT_MM));
  AddSequential(new frc::WaitCommand(0.25));
  AddSequential(new VisionScoringCmd(true));
  //Forward to ship
  AddSequential(new SetManipulatorIntakePowerCmd(1.0));
  AddSequential(new frc::WaitCommand(0.15));
  AddSequential(new HatchGripperContractCmd());
  AddSequential(new ManipulatorInCmd(0.3));
  //Drop off hatch
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_MAX_SAFE_HEIGHT_MANIPULATOR_VERTICAL - 5.0));
  AddParallel(new SetManipulatorAngleCmd(-90));
  //drop down to low height, return manipulator to middle
  AddSequential(new SilkyMotionCmd(std::vector<double> {-1000, -1600}, std::vector<double> {-45, 17}));
  //might be wrong
  AddSequential(new VisionHatchPickupGrp());
  //pick up the hatch
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_MAX_SAFE_HEIGHT_MANIPULATOR_VERTICAL - 5.0));
  AddParallel(new SetManipulatorAngleCmd(90));
  AddSequential(new SilkyMotionCmd(std::vector<double> {2000, 2500, -750, -1200, -750, -750}, std::vector<double> {0, -90, -75, 0, 5, -48}));
  //move to infront of rocket
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_MEDIUM_HATCH_HEIGHT_MM));
  AddSequential(new frc::WaitCommand(0.25));
  AddSequential(new VisionScoringCmd(true));
  //drop that hatch off again
  AddSequential(new CargoModeGrp());
}
