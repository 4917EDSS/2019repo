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
  logger.send(logger.DEBUGGING, "Joystick is the executive operator of elevator \n");
}

// Called repeatedly when this Command is scheduled to run
void ElevatorWithJoystickCmd::Execute() {
  std::shared_ptr<frc::Joystick> operatorJoystick = Robot::oi.getOperatorController();

  double elevatorStick = operatorJoystick->GetRawAxis(OPERATOR_ELEVATOR_AXIS);
	elevatorStick = pow(elevatorStick, 3);
  logger.send(logger.DEBUGGING, "The elevator is being used @ %f\n", elevatorStick);

  Robot::elevatorSub.setElevatorMotor(elevatorStick);

  double manipulatorStick = operatorJoystick->GetRawAxis(OPERATOR_MANIPULATOR_AXIS);
  manipulatorStick = pow(manipulatorStick, 3);
  logger.send(logger.DEBUGGING, "The manipulator is being used @ %f\n", manipulatorStick);

  Robot::elevatorSub.setWheels(manipulatorStick, manipulatorStick);

}

// Make this return true when this Command no longer needs to run execute()
bool ElevatorWithJoystickCmd::IsFinished() { 
  return false; 
  }

// Called once after isFinished returns true
void ElevatorWithJoystickCmd::End() {
  Robot::elevatorSub.setElevatorMotor(0.0);
  Robot::elevatorSub.setWheels(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ElevatorWithJoystickCmd::Interrupted() {
  ElevatorWithJoystickCmd::End();
}
