/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ElevatorWithJoystickCmd.h"
#include "OI.h"
#include "Robot.h"

ElevatorWithJoystickCmd::ElevatorWithJoystickCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void ElevatorWithJoystickCmd::Initialize() {
  currentManipulatorPosition = Robot::manipulatorSub.getFlipperAngle(); 
  logger.send(logger.ELEVATOR, "Robot flipper starting position is at %f \n", currentManipulatorPosition);
}

// Called repeatedly when this Command is scheduled to run
void ElevatorWithJoystickCmd::Execute() {
  std::shared_ptr<frc::Joystick> operatorJoystick = Robot::oi.getOperatorController();
  int shift = Robot::oi.getOperatorShiftState();

  // Right operator joystick vertical axis
  // In normal operation, this controls the elevator but when 'shift' buttons are selected,
  // it operates other mechanisms
  double verticalStick = operatorJoystick->GetRawAxis(OPERATOR_ELEVATOR_AXIS) * (-1); // Up is negative
  verticalStick = pow(verticalStick, 3);

  switch(shift) {
  case 0: // No shift, normal elevator operation
    // TODO: Replace with non-raw version (with slowdowns and limits)
    //logger.send(logger.ELEVATOR, "Call holdManFlip\n");
    Robot::manipulatorSub.holdManipulatorFlipper(currentManipulatorPosition);
    Robot::elevatorSub.setElevatorMotorRaw(verticalStick * 0.5);  // Limit power to 50%
    logger.send(logger.WITH_JOYSTICK_TRACE, "The elevator is being controlled @ %f\n", verticalStick);
    break;

  case 1: // Ball intake in/out flipper (handled in BallintakeWithJoystickCmd)  
    Robot::elevatorSub.setElevatorMotorRaw(0.0); 
    Robot::manipulatorSub.setFlipperPower(0.0);
    break;

  case 2: // Manipulator flip forward/backward
    Robot::elevatorSub.setElevatorMotorRaw(0.0); 
    Robot::manipulatorSub.setFlipperPower(verticalStick * 0.5);  // Limit power to 50%
    currentManipulatorPosition = Robot::manipulatorSub.getFlipperAngle();
    //logger.send(logger.ELEVATOR, "The manipulator flipper is being controlled @ %f\n", verticalStick);
    break;

  default:
    Robot::elevatorSub.setElevatorMotorRaw(0.0); 
    // Robot::elevatorSub.setManipulatorFlipperMotorSpeed(0.0);
    break;
  }


  // Left operator joystick vertical axis
  // Manipulator wheels in/out
  double manipulatorStick = operatorJoystick->GetRawAxis(OPERATOR_MANIPULATOR_AXIS);
  manipulatorStick = pow(manipulatorStick, 3);
  logger.send(logger.WITH_JOYSTICK_TRACE, "The manipulator wheels are being controlled @ %f\n", manipulatorStick);
  Robot::manipulatorSub.setIntakePower(manipulatorStick);
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorWithJoystickCmd::IsFinished() { 
  return false; 
  }

// Called once after isFinished returns true
void ElevatorWithJoystickCmd::End() {
  Robot::elevatorSub.setElevatorMotorSpeed(0.0);
  Robot::manipulatorSub.setFlipperPower(0.0);
  Robot::manipulatorSub.setIntakePower(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorWithJoystickCmd::Interrupted() {
  End();
}
