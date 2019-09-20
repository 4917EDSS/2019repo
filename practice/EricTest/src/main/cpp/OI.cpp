/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <frc/WPILib.h>
#include "commands/BallIntakeCmd.h"

OI::OI() {
  // Process operator interface input here.
  driverController.reset(new 	frc::Joystick(DRIVER_CONTROLLER_PORT));

  ballIntakeBtn.reset(new frc::JoystickButton(driverController.get(), BALL_INTAKE_BTN));
  ballIntakeBtn->WhenPressed(new BallIntakeCmd());
}

std::shared_ptr<frc::Joystick> OI::getDriverController() {
	return driverController;
}
