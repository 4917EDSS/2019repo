#include "commands/DriveStraightCmd.h"
#include "Robot.h"

DriveStraightCmd::DriveStraightCmd(double time, double power): time(time), power(power) {
  Requires(&Robot::drivetrainSub);
}

void DriveStraightCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::drivetrainSub.enableBalancerPID();
}

void DriveStraightCmd::Execute() {
  Robot::drivetrainSub.driverDriveStraight(power);
}

bool DriveStraightCmd::IsFinished() {
  if(TimeSinceInitialized() > time) {
    return true;
  }
  else {
    return false;
  }
}

void DriveStraightCmd::End() {
  Robot::drivetrainSub.drive(0,0);
  Robot::drivetrainSub.disableBalancerPID();
}

void DriveStraightCmd::Interrupted() {
  End();
}
