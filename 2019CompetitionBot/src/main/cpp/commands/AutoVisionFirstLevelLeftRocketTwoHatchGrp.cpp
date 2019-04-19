/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoVisionFirstLevelLeftRocketTwoHatchGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/VisionScoringCmd.h"
#include "commands/CargoModeGrp.h"
#include "commands/VisionHatchPickupGrp.h"

AutoVisionFirstLevelLeftRocketTwoHatchGrp::AutoVisionFirstLevelLeftRocketTwoHatchGrp() {
  AddSequential(new ExpandHatchGripperGrp());
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM));
  AddParallel(new SetManipulatorAngleCmd(0));
  AddSequential(new SilkyMotionCmd(std::vector<double> {1700, 400}, std::vector<double> {-70, 40}));
  // Vision to close-to-ship
  AddSequential(new VisionScoringCmd());
  AddSequential(new SilkyMotionCmd(std::vector<double> {-400}, std::vector<double> {0}));
  AddSequential(new SetManipulatorAngleCmd(90));
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_MEDIUM_HATCH_HEIGHT_MM));
  AddSequential(new SilkyMotionCmd(std::vector<double> {450}, std::vector<double> {0}));
  //Forward to ship
  AddSequential(new CargoModeGrp());
  //Drop off hatch
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM));
  AddParallel(new SetManipulatorAngleCmd(0));
  //drop down to low height, return manipulator to middle
  AddSequential(new SilkyMotionCmd(std::vector<double> {-500, -1000, -1000, -2000}, std::vector<double> {-45, 70, -25, 0}));
  //might be wrong
  AddParallel(new SetManipulatorAngleCmd(-90));
  AddSequential(new VisionHatchPickupGrp());
  //pick up the hatch
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM));
  AddParallel(new SetManipulatorAngleCmd(0));
  AddSequential(new SilkyMotionCmd(std::vector<double> {3400, 1500, 1000, 1000, 400}, std::vector<double> {0, 60, -45, -45, -110}));
  //move to infront of rocket
  AddSequential(new VisionScoringCmd());
  AddSequential(new SilkyMotionCmd(std::vector<double> {-400}, std::vector<double> {0}));
  AddSequential(new SetManipulatorAngleCmd(90));
  AddParallel(new SetElevatorToHeightCmd(ELEVATOR_MEDIUM_HATCH_HEIGHT_MM));
  AddSequential(new SilkyMotionCmd(std::vector<double> {450}, std::vector<double> {0}));
  //drop that hatch off again
  AddSequential(new CargoModeGrp());
}
