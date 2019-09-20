/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveWithJoystickCmd.h"
#include "Robot.h"
#include "subsystems/DrivetrainSub.h"

DriveWithJoystickCmd::DriveWithJoystickCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::drivetrainSub);
}

// Called just before this Command runs the first time
void DriveWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
  std::shared_ptr<frc::Joystick> driverJoystick = 
  Robot::m_oi.getDriverController();
  Robot::drivetrainSub.drive(driverJoystick->GetY(), driverJoystick->GetZ());

}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoystickCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void DriveWithJoystickCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoystickCmd::Interrupted() {}
