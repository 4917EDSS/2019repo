#include "commands/DriveWithIntakeMotorCmd.h"
#include "Robot.h"
DriveWithIntakeMotorCmd::DriveWithIntakeMotorCmd() {
  Requires(&Robot::ballIntakeSub);
}
void DriveWithIntakeMotorCmd::Initialize() {
  Robot::ballIntakeSub.setIntakeMotor(1.0);
}
void DriveWithIntakeMotorCmd::Execute() {}
bool DriveWithIntakeMotorCmd::IsFinished() { return false; }
void DriveWithIntakeMotorCmd::End() {}
void DriveWithIntakeMotorCmd::Interrupted() {
  End();
}