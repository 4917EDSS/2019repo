/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/VisionHighScoreGrp.h"
#include "commands/VisionScoringCmd.h"
#include "commands/CargoModeGrp.h"

VisionHighScoreGrp::VisionHighScoreGrp() {
  AddSequential(new VisionScoringCmd());
  AddSequential(new CargoModeGrp());
}
