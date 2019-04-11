/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SideBaseCmd.h"
#include "Robot.h"
#include "subsystems/ElevatorSub.h"
#include "subsystems/ManipulatorSub.h"


SideBaseCmd::SideBaseCmd(Command *onTrue, Command *onFalse) : frc::ConditionalCommand(onTrue, onFalse) {}

bool SideBaseCmd::Condition() {
  return Robot::manipulatorSub.getFlipperAngle() >= 0;
}