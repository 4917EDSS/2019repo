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
#include "commands/IntakeBallUntilLimitCmd.h"
#include "commands/KillEverythingCmd.h"
#include "commands/FlipManipulatorCmd.h"
#include "commands/TestButtonCmd.h"
#include "commands/MilkyScoreGrp.h"
#include "commands/MilkyManipulatorCmd.h"
#include "commands/ClimbCmdGroup.h"
#include "commands/SetElevatorandManipulatorCmd.h"
#include "commands/SetIntakeArmAngleCmd.h"

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

  OperatorKillBtn1.reset(new frc::JoystickButton(operatorController.get(), OPERATOR_KILL_ONE_BTN));
  OperatorKillBtn1->WhenPressed(new KillEverythingCmd());

  OperatorKillBtn2.reset(new frc::JoystickButton(operatorController.get(), OPERATOR_KILL_TWO_BTN));
  OperatorKillBtn2->WhenPressed(new KillEverythingCmd());

  DriverKillBtn1.reset(new frc::JoystickButton(driverController.get(), DRIVER_KILL_ONE_BTN));
  DriverKillBtn1->WhenPressed(new KillEverythingCmd());

  DriverKillBtn2.reset(new frc::JoystickButton(driverController.get(), DRIVER_KILL_TWO_BTN));
  DriverKillBtn2->WhenPressed(new KillEverythingCmd());


  milkyManipulatorBtn.reset(new frc::JoystickButton(driverController.get(),MILKY_MANIPULATOR_BTN));
  milkyManipulatorBtn->WhileHeld( new MilkyScoreGrp());

  resetIntakeBtn.reset(new frc::JoystickButton(operatorController.get(),RESET_INTAKE_BTN));
  resetIntakeBtn->WhenPressed(new SetIntakeArmAngleCmd(0));
      
  climbModeBtn.reset(new frc::JoystickButton(operatorController.get(), CLIMB_MODE_BTN));
  climbModeBtn->WhileHeld(new ClimbCmdGroup());

  TestBtn.reset(new frc::JoystickButton(operatorController.get(), TEST_BTN));
  TestBtn->WhileHeld(new TestButtonCmd());

  setManipulatorEncoderZeroBtn.reset(new frc::JoystickButton(operatorController.get(), SET_MANIPULATOR_ENCODER_ZERO_BTN));
  setManipulatorEncoderZeroBtn->WhenPressed(new SetElevatorandManipulatorCmd(0.0, 0.0));
}

std::shared_ptr<frc::Joystick> OI::getDriverController() {
	return driverController;
}

std::shared_ptr<frc::Joystick> OI::getOperatorController() {
  return operatorController;
}
