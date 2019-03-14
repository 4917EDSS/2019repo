/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/DriveWithJoystickCmd.h"
#include "OI.h"
#include "Robot.h"
#include "subsystems/BallIntakeSub.h"

constexpr double JOYSTICK_DEADBAND = 0.01;
constexpr double MAX_POWER = 1.0;

// Tune these parameters to change normal driving sensativity 
// (i.e. when using left and right joysticks)
constexpr double DRIVE_MINIMUM_OUTPUT_POWER = 0.0;	// 0.0 = not currently used
constexpr double DRIVE_POW_EXPONENT = 2.0;			// higher makes it less sensative at the low end but very sensative at the high end

// Tune these parameters to change rotate-only driving sensativity 
// (i.e. when only using the right joystick)
constexpr double ROTATE_MINIMUM_OUTPUT_POWER = 0.0;	// 0.0 = not currently used
constexpr double ROTATE_POW_EXPONENT = 2.0;			// higher makes it less sensative at the low end but very sensative at the high end


DriveWithJoystickCmd::DriveWithJoystickCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::drivetrainSub);
}

// Called just before this Command runs the first time
void DriveWithJoystickCmd::Initialize() {
	logger.send(logger.WITH_JOYSTICK_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoystickCmd::Execute() {
	
    std::shared_ptr<frc::Joystick> driverJoystick = Robot::oi.getDriverController();

    double rightStick = driverJoystick->GetZ();
    double leftStick = driverJoystick->GetY();

	// In climb mode the intake wheels are used as extra drive wheels
	if (Robot::inClimbMode) {
		if (fabs(leftStick) > JOYSTICK_DEADBAND) {
			Robot::ballIntakeSub.setIntakeWheelPower(leftStick);	// TODO:  Should this be exponential instead of linear?
		}
		else {
			Robot::ballIntakeSub.setIntakeWheelPower(0);
		}
	}

	// Set drivetrain motors based on left and right driver sticks
	
	// Calculate the power we would want to apply
	double drivePower;
	drivePower = fabs(leftStick);					// Take the sign away so we don't have deal with + vs - right now
	drivePower -= JOYSTICK_DEADBAND;				// Subtract off the deadband so we start at '0' power once we get past the deadband
	drivePower = pow(drivePower, DRIVE_POW_EXPONENT); 	// Take the magnitude and raise it to the definied power to get an exponential response to the joysticks
	drivePower += DRIVE_MINIMUM_OUTPUT_POWER;		// Add the minimum power that will make the machanism move (i.e. overcome static friction)
	drivePower = std::min(drivePower, MAX_POWER);	// Ensure that power isn't more than max
	drivePower *= (leftStick < 0) ? -1.0 : 1.0;		// Now that the math is done, re-add the sign (i.e. direction)

	double rotatePower;
	rotatePower = fabs(rightStick);					// Take the sign away so we don't have deal with + vs - right now
	rotatePower -= JOYSTICK_DEADBAND;				// Subtract off the deadband so we start at '0' power once we get past the deadband
	rotatePower = pow(rotatePower, ROTATE_POW_EXPONENT); 	// Take the magnitude and raise it to the definied power to get an exponential response to the joysticks
	rotatePower += ROTATE_MINIMUM_OUTPUT_POWER;		// Add the minimum power that will make the machanism move (i.e. overcome static friction)
	rotatePower = std::min(rotatePower, MAX_POWER);	// Ensure that power isn't more than max
	rotatePower *= (rightStick < 0) ? -1.0 : 1.0;	// Now that the math is done, re-add the sign (i.e. direction)

	if (fabs(leftStick) < JOYSTICK_DEADBAND) {
		// Left stick is idle.  Make sure that we disable "drive-straight".
		if (wasDrivingStraight > 0) {
			Robot::drivetrainSub.disableBalancerPID();
			wasDrivingStraight = 0;
		}

		// If right stick is also idle, stop the drivetrain
		if (fabs(rightStick) < JOYSTICK_DEADBAND) {
			Robot::drivetrainSub.drive(0, 0);
		}
		else {
			// Right stick is not idle so rotate the robot
			Robot::drivetrainSub.drive(rotatePower, -rotatePower);
		}
	}
	else if (fabs(rightStick) < JOYSTICK_DEADBAND) {
		// Right stick is idle and left stick is not.  We want to drive straight.
		if (wasDrivingStraight == 0) {
			// Get ready to drive straight.  Don't start now since AHRS might not have the correct heading angle yet
			timeSinceDrivingStraight = RobotController::GetFPGATime();
			wasDrivingStraight = 1;
		}
		if (((RobotController::GetFPGATime() - timeSinceDrivingStraight) >= AHRS_DELAY_TIME) && wasDrivingStraight == 1) {
			// Ready to use AHRS to keep us straight
			Robot::drivetrainSub.enableBalancerPID(Robot::drivetrainSub.getAngle()); //getAngle() should display above 360
			wasDrivingStraight = 2;
		}

		Robot::drivetrainSub.driverDriveStraight(-(drivePower));
	} 
	else {
		// Neither joystick is in the deadband so drive and turn.  Make sure that we disable "drive-straight".
		if (wasDrivingStraight > 0) {
			Robot::drivetrainSub.disableBalancerPID();
			wasDrivingStraight = 0;
		}

		double fastSide = std::max(fabs(drivePower), fabs(rotatePower));
    	double slowSide = -drivePower + fabs(rotatePower) * drivePower;

		if (leftStick < 0) { //if leftStick < 0, leftStick is pushed up
			if (rightStick < 0) { // 
				Robot::drivetrainSub.drive(slowSide , fastSide);
			} 
			else {
				Robot::drivetrainSub.drive(fastSide, slowSide);
			}
		} 
		else {
			if (rightStick > 0) {
				Robot::drivetrainSub.drive(slowSide, -fastSide);
			} 
			else {
				Robot::drivetrainSub.drive(-fastSide, slowSide);
			}
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoystickCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void DriveWithJoystickCmd::End() {
	Robot::drivetrainSub.drive(0, 0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoystickCmd::Interrupted() {
	End();
}
