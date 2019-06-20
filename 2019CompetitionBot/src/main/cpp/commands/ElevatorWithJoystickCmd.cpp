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
void ElevatorWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ElevatorWithJoystickCmd::Execute() {
  std::shared_ptr<frc::Joystick> operatorJoystick = Robot::oi.getOperatorController();
  int shift = Robot::oi.getOperatorShiftState();

  // Right operator joystick vertical axis
  // In normal operation, this controls the elevator but when 'shift' buttons are selected,
  // it operates other mechanisms
  double power = operatorJoystick->GetRawAxis(OPERATOR_ELEVATOR_AXIS) * (-1); // Up is negative
  power = pow(power, 3);

  if(Robot::demoMode) {
		// Slow down the drivetrain for inexperienced drivers
		power *= 0.5;
	}

  switch(shift) {
  case 0: // No shift, normal elevator operation
    Robot::elevatorSub.setElevatorHeight(ELEVATOR_MODE_MANUAL, power, 0);
    //logger.send(logger.WITH_JOYSTICK_TRACE, "The elevator is being controlled @ %f\n", power);
    break;

  // If the joystick is controlling something else, pass in a power of 0.
  // This will tell the state machine to hold the current position.
  case 1: // Ball intake in/out flipper (handled in BallintakeWithJoystickCmd)
  case 2: // Manipulator flip forward/backward (handled in ManipulatorWithJoystickCmd)
  default:
    Robot::elevatorSub.setElevatorHeight(ELEVATOR_MODE_MANUAL, 0, 0);
    break;
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorWithJoystickCmd::IsFinished() { 
  return false; 
  }

// Called once after isFinished returns true
void ElevatorWithJoystickCmd::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorWithJoystickCmd::Interrupted() {
  End();
}
