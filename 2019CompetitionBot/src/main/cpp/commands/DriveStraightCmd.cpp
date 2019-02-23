#include "commands/DriveStraightCmd.h"
#include "Robot.h"

DriveStraightCmd::DriveStraightCmd(double distance): distance(distance) {
  Requires(&Robot::drivetrainSub);
}
void DriveStraightCmd::Initialize() {
  Robot::drivetrainSub.resetAHRS();
}
void DriveStraightCmd::Execute() {
  double angle = Robot::drivetrainSub.getAngle();
  Robot::drivetrainSub.drive(0.5 - angle,0.5 + angle);
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);
}
bool DriveStraightCmd::IsFinished() {
  double distanceTraveled=(Robot::drivetrainSub.GetLeftEncoder()+Robot::drivetrainSub.GetRightEncoder())/2;
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
