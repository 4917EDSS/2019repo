/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveWithJoystickCmd.h"
#include "OI.h"
#include "Robot.h"

DriveWithJoystickCmd::DriveWithJoystickCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::drivetrainSub);
}

// Called just before this Command runs the first time
void DriveWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
    std::shared_ptr<frc::Joystick> driverJoystick = Robot::oi.getDriverController();

    double rightStick = driverJoystick->GetZ();
    double leftStick = driverJoystick->GetY();
    rightStick = pow(rightStick, 3);
    leftStick = pow(leftStick, 3);
    double fastSide = std::max(fabs(leftStick), fabs(rightStick));
    double slowSide = -leftStick + fabs(rightStick) * leftStick;

	if (leftStick < 0.01 && leftStick > -0.01) {

		Robot::drivetrainSub.drive(rightStick, -rightStick);

	} else {

		if (leftStick < 0) {
			if (rightStick < 0) {
				Robot::drivetrainSub.drive(slowSide , fastSide);
			} else {
			
				Robot::drivetrainSub.drive(fastSide, slowSide );
			}
		} else {
			if (rightStick > 0) {
				Robot::drivetrainSub.drive(slowSide, -fastSide);
			} else {
				Robot::drivetrainSub.drive(-fastSide, slowSide);
			}
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoystickCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void DriveWithJoystickCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoystickCmd::Interrupted() {}
