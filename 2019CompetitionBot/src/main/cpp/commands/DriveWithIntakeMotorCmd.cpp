
#include "commands/DriveWithIntakeMotorCmd.h"
#include "Robot.h"


DriveWithIntakeMotorCmd::DriveWithIntakeMotorCmd() {
  Requires(&Robot::ballIntakeSub);
}

void DriveWithIntakeMotorCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::ballIntakeSub.setIntakeWheelPower(1.0);
}

void DriveWithIntakeMotorCmd::Execute() {}

bool DriveWithIntakeMotorCmd::IsFinished() { return false; }

void DriveWithIntakeMotorCmd::End() {
  Robot::ballIntakeSub.setIntakeWheelPower(0.0);
}

void DriveWithIntakeMotorCmd::Interrupted() {
  End();
}
