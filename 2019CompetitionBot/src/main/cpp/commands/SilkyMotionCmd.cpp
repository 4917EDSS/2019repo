#include "Commands/SilkyMotionCmd.h"
#include "subsystems/DrivetrainSub.h"
#include <iostream>
#include <algorithm>
#include <math.h>

const double MAX_LIN_ACCEL=2200.0;
const double MAX_LIN_DECEL=3000.0;
const double MAX_LIN_VEL=1800.0;
const double MAX_ANG_ACCEL=800.0;

const double P_DIS=0.0015;
const double D_DIS=0.000002;
const double A_DIS=0.125*(0.8/MAX_LIN_VEL);
const double V_DIS=0.6/MAX_LIN_VEL;
const double P_ANG=0.013;
//When we drove at these speeds we were at 1.0 power we got 360 degrees per second
const double V_ANG=1.1*(1.0/360.0);

constexpr float DRIVE_DISTANCE_TOLERANCE = 15.0;
constexpr float DISTANCE_SPEED_TOLERANCE = 10.0;
constexpr float DRIVE_TURN_TOLERANCE = 2.0;
constexpr float DRIVE_RATE_TOLERANCE = 1;

SilkyMotionCmd::SilkyMotionCmd(std::vector<double> dis, std::vector<double> ang) : smm(dis, ang,
		 MAX_LIN_ACCEL, MAX_LIN_DECEL, MAX_LIN_VEL, MAX_ANG_ACCEL) {
	Requires(&Robot::drivetrainSub);
	prevTime = 0;
	prevError = 0;
}

SilkyMotionCmd::SilkyMotionCmd(std::vector<double> dis, std::vector<double> ang, double decel) : smm(dis, ang,
		 MAX_LIN_ACCEL, decel, MAX_LIN_VEL, MAX_ANG_ACCEL) {
	Requires(&Robot::drivetrainSub);
	prevTime = 0;
	prevError = 0;
}

// Called just before this Command runs the first time
void SilkyMotionCmd::Initialize() {
	Robot::drivetrainSub.resetAHRS();
	Robot::drivetrainSub.SetDrivetrainEncoderZero();
	prevTime = 0;
	prevError = 0;
}


// Called repeatedly when this Command is scheduled to run
void SilkyMotionCmd::Execute() {
	double currTime = TimeSinceInitialized();
	PathInfo p = smm.getCurrentPathInfo(currTime);

	double currDis=(Robot::drivetrainSub.getLeftEncoder() + Robot::drivetrainSub.getRightEncoder())/2;
	double currError=p.dis-currDis;
	double errorDer=(currError-prevError)/(currTime-prevTime);
	double angErr=p.ang-Robot::drivetrainSub.getAngle();
	
	double forward = P_DIS*currError+D_DIS*errorDer+A_DIS*p.lin_accel+V_DIS*p.lin_vel;
	if (fabs(forward) > 1.0) {
		std::cout << "SATURATION ON DRIVE FORWARD: " << forward << std::endl;
		std::cout << "P " << P_DIS*currError << " D " << D_DIS*errorDer<< " A " << A_DIS*p.lin_accel<< " V " << V_DIS*p.lin_vel << std::endl;
		if (forward > 1.0) {
			forward = 1.0;
		} else if (forward < -1.0) {
			forward = -1.0;
		}
	}

	Robot::drivetrainSub.drive(forward + P_ANG*angErr + V_ANG*p.ang_vel,
			     forward - P_ANG*angErr - V_ANG*p.ang_vel);
	//logger.send(logger.DEBUG, "silk:, %f, %f, %f, %f, %f, %f, %f, %f, %f", currTime, currError, angErr, P_DIS*currError, D_DIS*errorDer, A_DIS*p.lin_accel, V_DIS*p.lin_vel, P_ANG*angErr, V_ANG*p.ang_vel);
	SmartDashboard::PutNumber("Current Error: ", currError);
	SmartDashboard::PutNumber("Angle Error: ", angErr);
	prevError = currError;
	prevTime = currTime;
}

// Make this return true when this Command no longer needs to run execute()
bool SilkyMotionCmd::IsFinished() {
	double currTime = TimeSinceInitialized();
	if (currTime > smm.getTotalTime()+0.1) {
		return true;
	} else if (currTime >= smm.getTotalTime()){
		PathInfo p = smm.getCurrentPathInfo(currTime);
		double currDis=(Robot::drivetrainSub.getLeftEncoder() + Robot::drivetrainSub.getRightEncoder())/2;
		if(fabs(p.dis - currDis) < DRIVE_DISTANCE_TOLERANCE && fabs(Robot::drivetrainSub.getVelocity()) < DISTANCE_SPEED_TOLERANCE && fabs(p.ang - Robot::drivetrainSub.getAngle()) < DRIVE_TURN_TOLERANCE){
			return true;
		}
	}

	return false;
}

  // Called once after isFinished returns true
void SilkyMotionCmd::End() {
  Robot::drivetrainSub.drive(0,0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SilkyMotionCmd::Interrupted() {
  End();
}
