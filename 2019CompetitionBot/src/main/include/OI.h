/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <frc/WPILib.h>

/*
 * ON LOGITECH F310 CONTROLLER:
 * X = 1            (Blue)
 * A = 2            (Green)
 * B = 3            (Red)
 * Y = 4            (Yellow)
 * LB = 5           (Left-Bumper: top button)
 * RB = 6           (Right-Bumper: top button)
 * LT = 7           (Left-Trigger: bottom button)
 * RT = 8           (Right-Trigger: bottom button)
 * Select/Back = 9  (Above left joystick)
 * Start = 10       (Above right joytsick)
 * L3 = 11          (Press left joystick)
 * R3 = 12          (Press right joystick)
 * 
 * Left Joystick Vertical Axis = 1
 * Left Joystick Horizontal Axis = 0
 * Right Joystick Vertical Axis = 3
 * Right Joystick Horizontal Axis = 2
 */

// Controllers (aka Joysticks)
constexpr int DRIVER_CONTROLLER_PORT = 0;
constexpr int OPERATOR_CONTROLLER_PORT = 1;

// Driver Buttons
constexpr int DRIVE_TO_VISION_TARGET_BTN = 1;
// 2 - free
// 3 - free
// 4 - free
constexpr int CLIMB_BTN = 5;
constexpr int EXTEND_CLIMB_BARS_BTN = 6;
constexpr int REVERSE_CLIMB_BTN = 7;
constexpr int RETRACT_CLIMB_BARS_BTN = 8;
constexpr int MILKY_MANIPULATOR_BTN = 9;
constexpr int TEST_BTN = 10;
constexpr int DRIVER_KILL_ONE_BTN = 11;
constexpr int DRIVER_KILL_TWO_BTN = 12;

// Driver Axes
//...

// Operator Buttons
constexpr int ELEVATOR_TO_CARGO_SHIP_HEIGHT_BTN = 1;
constexpr int ELEVATOR_TO_LOW_HEIGHT_BTN = 2;
constexpr int ELEVATOR_TO_MEDIUM_HEIGHT_BTN = 3;
constexpr int ELEVATOR_TO_HIGH_HEIGHT_BTN = 4;
constexpr int HATCH_MODE_BTN = 5;
constexpr int CARGO_MODE_BTN = 6;
constexpr int FLIP_MANIPULATOR_BTN = 7;
constexpr int INTAKE_HATCH_OR_CARGO_BTN = 8;
constexpr int MANIPULATOR_TO_VERTICAL_BTN = 9;
constexpr int MULTI_COMMAND_1_BTN = 10;
constexpr int OPERATOR_KILL_ONE_BTN = 11;
constexpr int OPERATOR_KILL_TWO_BTN = 12;

// Operator Axes
constexpr int OPERATOR_MANIPULATOR_AXIS = 1;
constexpr int OPERATOR_INTAKE_ARM_AXIS = 2;
constexpr int OPERATOR_ELEVATOR_AXIS = 3;


// POV D-Pad
constexpr int POV_INACTIVE = -1;
constexpr int POV_UP = 0;
constexpr int POV_UP_RIGHT = 45;
constexpr int POV_RIGHT = 90;
constexpr int POV_RIGHT_DOWN = 135;
constexpr int POV_DOWN = 180;
constexpr int POV_DONW_LEFT = 225;
constexpr int POV_LEFT = 270;
constexpr int POV_LEFT_UP = 315;

class OI {
 public:
  OI();

  std::shared_ptr<frc::Joystick> getDriverController();
  std::shared_ptr<frc::Joystick> getOperatorController();
  int getOperatorShiftState();

 private:
    std::shared_ptr<frc::Joystick> driverController;
    std::shared_ptr<frc::Joystick> operatorController;

    // Driver controller buttons
    std::shared_ptr<frc::JoystickButton> driveToVisionTargetBtn;
    std::shared_ptr<frc::JoystickButton> climbBtn;
    std::shared_ptr<frc::JoystickButton> extendClimbBarsBtn;
    std::shared_ptr<frc::JoystickButton> reverseClimbBtn;
    std::shared_ptr<frc::JoystickButton> retractClimbBarsBtn;
    std::shared_ptr<frc::JoystickButton> milkyManipulatorBtn;
    std::shared_ptr<frc::JoystickButton> testBtn;
    std::shared_ptr<frc::JoystickButton> driverKillBtn1;
    std::shared_ptr<frc::JoystickButton> driverKillBtn2;
    

    // Operator controller buttons
    std::shared_ptr<frc::JoystickButton> elevatorToCargoShipHeightBtn;
    std::shared_ptr<frc::JoystickButton> elevatorToLowHeightBtn;
    std::shared_ptr<frc::JoystickButton> elevatorToMediumHeightBtn;
    std::shared_ptr<frc::JoystickButton> elevatorToHighHeightBtn;
    std::shared_ptr<frc::JoystickButton> hatchModeBtn;
    std::shared_ptr<frc::JoystickButton> cargoModeBtn;
    std::shared_ptr<frc::JoystickButton> flipManipulatorBtn;
    std::shared_ptr<frc::JoystickButton> intakeHatchOrCargoBtn;
    std::shared_ptr<frc::JoystickButton> manipulatorToVerticalBtn;
    std::shared_ptr<frc::JoystickButton> multiCommand1Btn;    
    std::shared_ptr<frc::JoystickButton> operatorKillBtn1;
    std::shared_ptr<frc::JoystickButton> operatorKillBtn2;
};
