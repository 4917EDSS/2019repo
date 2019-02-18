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
 * Select/Back = 9
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
constexpr int DRIVER_KILL_ONE_BTN = 11;
constexpr int DRIVER_KILL_TWO_BTN = 12;
constexpr int MILKY_MANIPULATOR_BTN= 9;

//Operator
constexpr int HATCH_CONTRACT_BTN = 3;
constexpr int SET_INTAKE_MOTOR_BTN = 2;
constexpr int OPERATOR_KILL_ONE_BTN = 11;
constexpr int OPERATOR_KILL_TWO_BTN = 12;

//constexpr int CLIMB_MODE_BTN = 5;
constexpr int SHIFTER_LOW_WHILE_HELD = 5;
constexpr int TEST_BTN = 9;
constexpr int SET_MANIPULATOR_ENCODER_ZERO_BTN = 8;
constexpr int RESET_INTAKE_BTN = 7;
constexpr int TOGGLE_HATCH_PANEL_GRABBER = 10;
constexpr int INTAKE_BALL_BTN=4;

constexpr int OPERATOR_MANIPULATOR_AXIS = 1;
constexpr int OPERATOR_INTAKE_ARM_AXIS = 2;
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
    std::shared_ptr<frc::JoystickButton> resetIntakeBtn;
    std::shared_ptr<frc::JoystickButton> intakeBallBtn;
    std::shared_ptr<frc::JoystickButton> climbModeBtn;
    std::shared_ptr<frc::JoystickButton> TestBtn;
    std::shared_ptr<frc::JoystickButton> setManipulatorEncoderZeroBtn;
    std::shared_ptr<frc::JoystickButton> toggleHatchPanelGrabberBtn;
    std::shared_ptr<frc::JoystickButton> shifterLowWhileHeldBtn;
};
