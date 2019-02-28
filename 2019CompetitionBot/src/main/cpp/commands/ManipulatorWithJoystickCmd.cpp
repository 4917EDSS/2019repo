/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ManipulatorWithJoystickCmd.h"
#include "OI.h"
#include "Robot.h"

ManipulatorWithJoystickCmd::ManipulatorWithJoystickCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::manipulatorSub);
}

// Called just before this Command runs the first time
void ManipulatorWithJoystickCmd::Initialize() {
  logger.send(logger.WITH_JOYSTICK_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
}

// Called repeatedly when this Command is scheduled to run
void ManipulatorWithJoystickCmd::Execute() {
  std::shared_ptr<frc::Joystick> operatorJoystick = Robot::oi.getOperatorController();
  int shift = Robot::oi.getOperatorShiftState();

  // Right operator joystick vertical axis
  // In normal operation, this controls the elevator but when 'shift' buttons are selected,
  // it operates other mechanisms
  double verticalStick = operatorJoystick->GetRawAxis(OPERATOR_ELEVATOR_AXIS) * (-1); // Up is negative
  verticalStick = pow(verticalStick, 3);

  switch(shift) {
    case 0: // Elevator (handled in ElevatorWithJoystickCmd)
    case 1: // Ball intake in/out flipper (handled in BallintakeWithJoystickCmd)  
      Robot::manipulatorSub.setFlipperPower(0.0);
      break;
    
    case 2: // Manipulator flip forward/backward
      Robot::manipulatorSub.setFlipperPower(verticalStick * 0.5);  // Limit power to 50%
      break;

    default:
      break;
  }

  // Left operator joystick vertical axis
  // Manipulator wheels in/out
  double manipulatorStick = operatorJoystick->GetRawAxis(OPERATOR_MANIPULATOR_AXIS);
  manipulatorStick = pow(manipulatorStick, 3);
  Robot::manipulatorSub.setIntakePower(manipulatorStick);
}

// Make this return true when this Command no longer needs to run execute()
bool ManipulatorWithJoystickCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void ManipulatorWithJoystickCmd::End() {
  Robot::manipulatorSub.setFlipperPower(0.0);
  Robot::manipulatorSub.setIntakePower(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ManipulatorWithJoystickCmd::Interrupted() {}
