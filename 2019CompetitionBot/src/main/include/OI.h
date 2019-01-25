/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>

/*
 * ON LOGITECH CONTROLLER:
 * X = 1
 * A = 2
 * B = 3
 * Y = 4
 * LB = 5
 * RB = 6
 * LT = 7
 * RT = 8
 * Select = 9
 * Start = 10
 * L3 = 11
 * R3 = 12
 * Left Vertical = 1
 * Left Horizontal = 0
 * Right Vertical = 3
 * Right Horizontal = 2
 *
 */

constexpr int DRIVER_CONTROLLER_PORT = 0;
constexpr int OPERATOR_CONTROLLER_PORT = 1;

//Driver
constexpr int DRIVER_KILL_BUTTON_ONE = 11;
constexpr int DRIVER_KILL_BUTTON_TWO = 12;
constexpr int MILKY_MANIPULATOR_BUTTON=13;
//Operator
constexpr int HATCH_CONTRACT_BTN = 3;
constexpr int SET_INTAKE_MOTOR_BTN = 2;
constexpr int FLIP_BALL_INTAKE_TOGGLE_BTN = 1;
constexpr int OPERATOR_KILL_BUTTON_ONE = 11;
constexpr int OPERATOR_KILL_BUTTON_TWO = 12;

constexpr int OPERATOR_ELEVATOR_AXIS = 3;

class OI {
 public:
  OI();

  std::shared_ptr<frc::Joystick> getDriverController();
  std::shared_ptr<frc::Joystick> getOperatorController();

 private:
    std::shared_ptr<frc::Joystick> driverController;
    std::shared_ptr<frc::Joystick> operatorController;
    std::shared_ptr<frc::JoystickButton> hatchContractBtn;
    std::shared_ptr<frc::JoystickButton> IntakeUntilLimitBtn;
    std::shared_ptr<frc::JoystickButton> OperatorKillBtn1;
    std::shared_ptr<frc::JoystickButton> OperatorKillBtn2;
    std::shared_ptr<frc::JoystickButton> DriverKillBtn1;
    std::shared_ptr<frc::JoystickButton> DriverKillBtn2;
    std::shared_ptr<frc::JoystickButton> ballFlipperToggleBtn;
    std::shared_ptr<frc::JoystickButton> milkyManipulatorBtn;
};
