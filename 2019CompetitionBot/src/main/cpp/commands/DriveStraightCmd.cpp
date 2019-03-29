#include "commands/DriveStraightCmd.h"
#include "Robot.h"

DriveStraightCmd::DriveStraightCmd(double distance, double power): distance(distance), power(power) {
  Requires(&Robot::drivetrainSub);
}

void DriveStraightCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::drivetrainSub.enableBalancerPID(Robot::drivetrainSub.getAngle());
  Robot::drivetrainSub.driverDriveStraight(power);
}

void DriveStraightCmd::Execute() {
}

bool DriveStraightCmd::IsFinished() {
  double distanceTraveled=(Robot::drivetrainSub.getLeftEncoder() + Robot::drivetrainSub.getRightEncoder()) / 2;
  if (distanceTraveled >= distance) {
    return true;
  }
  return false;
}

void DriveStraightCmd::End() {
  Robot::drivetrainSub.drive(0,0);
}

void DriveStraightCmd::Interrupted() {
  End();
}
