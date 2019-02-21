/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/BallIntakeWithJoystickCmd.h"
#include "Robot.h"

BallIntakeWithJoystickCmd::BallIntakeWithJoystickCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);
}

// Called just before this Command runs the first time
void BallIntakeWithJoystickCmd::Initialize() {
  logger.send(logger.WITH_JOYSTICK_TRACE, "Joystick is the executive operator of ball intake \n");
}

// Called repeatedly when this Command is scheduled to run
void BallIntakeWithJoystickCmd::Execute() {
  std::shared_ptr<frc::Joystick> operatorJoystick = Robot::oi.getOperatorController();
  int shift = Robot::oi.getOperatorShiftState();

  // Right operator joystick vertical axis
  // In normal operation, this controls the elevator but when 'shift' buttons are selected,
  // it operates other mechanisms
  double verticalStick = operatorJoystick->GetRawAxis(OPERATOR_ELEVATOR_AXIS) * (-1); // Up is negative
  verticalStick = pow(verticalStick, 3);

  switch(shift) {
  case 0: // No shift, normal elevator operation (handled in ElevatorWithJoystickCmd)
    Robot::ballIntakeSub.setIntakeArmMotor(0.0, false);
    break;

  case 1: // Ball intake in/out flipper
    Robot::ballIntakeSub.setIntakeArmMotor(verticalStick * 0.5, false);  // Limit power to 50%
    logger.send(logger.WITH_JOYSTICK_TRACE, "The ball intake flipper is being controlled @ %f\n", verticalStick);
    break;

  case 2: // Manipulator flip forward/backward (handled in ElevatorWithJoystickCmd)
    Robot::ballIntakeSub.setIntakeArmMotor(0.0, false);
    break;

  default:
    Robot::ballIntakeSub.setIntakeArmMotor(0.0, false);
    break;
  }
}

// Make this return true when this Command no longer needs to run execute()
bool BallIntakeWithJoystickCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void BallIntakeWithJoystickCmd::End() {
  Robot::ballIntakeSub.setIntakeArmMotor(0.0, false);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void BallIntakeWithJoystickCmd::Interrupted() {
  End();
}
