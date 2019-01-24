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
#include "Commands/CloseHatchPickupCmd.h"
#include "Commands/IntakeWhileHeldCmd.h"
#include "commands/IntakeBallUntilLimitCmd.h"
#include "commands/KillEverythingCmd.h"
#include "commands/FlipFlipperCmd.h"

OI::OI() {
  // Process operator interface input here.
  driverController.reset(new frc::Joystick(DRIVER_CONTROLLER_PORT));
  driverController->SetXChannel(0);
  driverController->SetYChannel(1);
  driverController->SetZChannel(2);
  driverController->SetThrottleChannel(3);

  operatorController.reset(new frc::Joystick(OPERATOR_CONTROLLER_PORT));
  operatorController->SetXChannel(0);
  operatorController->SetYChannel(1);
  operatorController->SetZChannel(2);
  operatorController->SetThrottleChannel(3);

  hatchContractBtn.reset(new frc::JoystickButton(operatorController.get(), HATCH_CONTRACT_BTN));
  hatchContractBtn->WhileHeld(new CloseHatchPickupCmd());

  IntakeUntilLimitBtn.reset(new frc::JoystickButton(operatorController.get(), SET_INTAKE_MOTOR_BTN));
  IntakeUntilLimitBtn->WhenPressed(new IntakeBallUntilLimitCmd());

  OperatorKillBtn1.reset(new frc::JoystickButton(operatorController.get(), OPERATOR_KILL_BUTTON_ONE));
  OperatorKillBtn1->WhenPressed(new KillEverythingCmd());

  OperatorKillBtn2.reset(new frc::JoystickButton(operatorController.get(), OPERATOR_KILL_BUTTON_TWO));
  OperatorKillBtn2->WhenPressed(new KillEverythingCmd());

  DriverKillBtn1.reset(new frc::JoystickButton(driverController.get(), DRIVER_KILL_BUTTON_ONE));
  DriverKillBtn1->WhenPressed(new KillEverythingCmd());

  DriverKillBtn2.reset(new frc::JoystickButton(driverController.get(), DRIVER_KILL_BUTTON_TWO));
  DriverKillBtn2->WhenPressed(new KillEverythingCmd());

  ballFlipperToggleBtn.reset(new frc::JoystickButton(operatorController.get(), FLIP_BALL_INTAKE_TOGGLE_BTN));
  ballFlipperToggleBtn->WhenPressed(new FlipFlipperCmd(FlipFlipperCmd::FlipperDirection::toggle));
  milkyMonipulaterBtn.reset(new frc::JoystickButton(driverController.get(),MILKY_MONIPULATER_BUTTON));
  milkyMonipulaterBtn->WhileHeld( new MilkyMonipulaterCmd());
}

std::shared_ptr<frc::Joystick> OI::getDriverController() {
	return driverController;
}

std::shared_ptr<frc::Joystick> OI::getOperatorController() {
  return operatorController;
}
