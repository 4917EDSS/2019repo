#include "commands/DriveWithIntakeMotorCmd.h"
#include "Robot.h"


DriveWithIntakeMotorCmd::DriveWithIntakeMotorCmd() {
  Requires(&Robot::ballIntakeSub);
}

void DriveWithIntakeMotorCmd::Initialize() {
  Robot::ballIntakeSub.setIntakeWheelsMotorSpeed(1.0);
}

void DriveWithIntakeMotorCmd::Execute() {}

bool DriveWithIntakeMotorCmd::IsFinished() { return false; }

void DriveWithIntakeMotorCmd::End() {
  Robot::ballIntakeSub.setIntakeWheelsMotorSpeed(0.0);
}

void DriveWithIntakeMotorCmd::Interrupted() {
  End();
}
