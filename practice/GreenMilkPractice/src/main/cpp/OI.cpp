/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <frc/WPILib.h>
#include "Robot.h"
#include "subsystems/BallIntakeSub.h"

OI::OI() {
  // Process operator interface input here.
  controller.reset(new frc::Joystick(CONTROLLER_PORT));
  controller->SetXChannel(0);
  controller->SetYChannel(1);
  controller->SetZChannel(2);
  controller->SetThrottleChannel(3);

}

