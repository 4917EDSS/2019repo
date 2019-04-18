#include "commands/DriveStraightCmd.h"
#include "Robot.h"

DriveStraightCmd::DriveStraightCmd(double time, double power): time(time), power(power) {
  Requires(&Robot::drivetrainSub);
}

void DriveStraightCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::drivetrainSub.drive(power, power);
}

void DriveStraightCmd::Execute() {
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
}

void DriveStraightCmd::Interrupted() {
  End();
}
