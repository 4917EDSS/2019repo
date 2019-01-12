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

  double verticalStick = operatorJoystick->GetRawAxis(OPERATOR_ELEVATOR_AXIS);
	verticalStick = pow(verticalStick, 3);

  Robot::elevatorSub.SetElevatorMotor(verticalStick);
}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorWithJoystickCmd::IsFinished() { 
  return false; 
  }

// Called once after isFinished returns true
void ElevatorWithJoystickCmd::End() {
  Robot::elevatorSub.SetElevatorMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorWithJoystickCmd::Interrupted() {
  ElevatorWithJoystickCmd::End();
}
