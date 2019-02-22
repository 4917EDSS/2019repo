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
#include "commands/KillEverythingCmd.h"
#include "commands/TestButtonCmd.h"
#include "commands/MilkyScoreGrp.h"
#include "commands/MilkyManipulatorCmd.h"
#include "commands/ClimbCmdGroup.h"
#include "commands/SetElevatorandManipulatorCmd.h"
#include "commands/SetIntakeArmAngleCmd.h"
#include "commands/ToggleHatchGripperCmd.h"
#include "commands/IntakeBallFromRobotCmd.h"
#include "commands/IntakeBallGrp.h"
#include "commands/MultiButton1Cmd.h"
#include "commands/HatchModeGrp.h"
#include "commands/CargoModeGrp.h"

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

  hatchModeBtn.reset(new frc::JoystickButton(operatorController.get(), HATCH_MODE_BTN));
  hatchModeBtn->WhenPressed(new HatchModeGrp());

  cargoModeBtn.reset(new frc::JoystickButton(operatorController.get(), CARGO_MODE_BTN));
  cargoModeBtn->WhenPressed(new CargoModeGrp());

  //IntakeUntilLimitBtn.reset(new frc::JoystickButton(operatorController.get(), SET_INTAKE_MOTOR_BTN));
  //IntakeUntilLimitBtn->WhenPressed(new IntakeBallFromRobotCmd());

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

  intakeBallBtn.reset(new frc::JoystickButton(operatorController.get(), INTAKE_BALL_BTN));
  intakeBallBtn->WhenPressed(new IntakeBallGrp());

  
  climbModeBtn.reset(new frc::JoystickButton(operatorController.get(), CLIMB_MODE_BTN));
  climbModeBtn->WhileHeld(new ClimbCmdGroup());

  TestBtn.reset(new frc::JoystickButton(operatorController.get(), TEST_BTN));
  TestBtn->WhileHeld(new TestButtonCmd());

  setManipulatorEncoderZeroBtn.reset(new frc::JoystickButton(operatorController.get(), SET_MANIPULATOR_ENCODER_ZERO_BTN));
  setManipulatorEncoderZeroBtn->WhenPressed(new SetElevatorandManipulatorCmd(0.0, 0.0));

  toggleHatchPanelGrabberBtn.reset(new frc::JoystickButton(operatorController.get(), TOGGLE_HATCH_PANEL_GRABBER));
  toggleHatchPanelGrabberBtn->WhenPressed(new ToggleHatchGripperCmd());

  multiCommand1Btn.reset(new frc::JoystickButton(operatorController.get(), MULTI_COMMAND_1_BUTTON));
  multiCommand1Btn->WhenPressed(new MultiButton1Cmd());
}

std::shared_ptr<frc::Joystick> OI::getDriverController() {
	return driverController;
}

std::shared_ptr<frc::Joystick> OI::getOperatorController() {
  return operatorController;
}

// Use the POV (i.e. D-Pad) as a command modifier
int OI::getOperatorShiftState() {
  int shift = 0;

  // POV position is reported in degrees with 0 deg being up and increasing clockwise
  // Positions between Up/down/left/right are ignored (i.e. 45 deg)
  switch(operatorController->GetPOV()) {
  case -1: // D-pad not pressed
    shift = 0;
    break;
  case 0:    // Up
    shift = 1;
    break; 
  case 90:  // Right
    shift = 2;
    break;
  case 180: // Down
    shift = 3;
    break;
  case 270: // Left
    shift = 4;
    break;
  }

  return shift;
}
