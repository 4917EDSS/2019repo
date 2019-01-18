#include "commands/DriveStraightCmd.h"
#include "Robot.h"

DriveStraightCmd::DriveStraightCmd(double distance): distance(distance) {
  Requires(&Robot::drivetrainSub);
}
void DriveStraightCmd::Initialize() {
  Robot::drivetrainSub.resetAHRS();
}
void DriveStraightCmd::Execute() {
  if (Robot::drivetrainSub.getAngle() > 0) {
    Robot::drivetrainSub.drive(0,0.2);
  }
  else if( Robot::drivetrainSub.getAngle() < 0) {
        Robot::drivetrainSub.drive(0.2,0);
  }
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
void DriveStraightCmd::Interrupt() {
  End();
}