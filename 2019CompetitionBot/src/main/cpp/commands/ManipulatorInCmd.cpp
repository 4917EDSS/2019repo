#include "commands/ManipulatorInCmd.h"
#include "Robot.h"
ManipulatorInCmd::ManipulatorInCmd (double intakeTime) : intakeTime(intakeTime) {
  Requires(&Robot::manipulatorSub);
}
ManipulatorInCmd::ManipulatorInCmd() {
  Requires(&Robot::manipulatorSub);
  double intakeTime=1.0;
}

void ManipulatorInCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::manipulatorSub.setIntakePower(-0.5);
  Robot::inBallMode = true;
}

void ManipulatorInCmd::Execute() {}

bool ManipulatorInCmd::IsFinished() {
  if (TimeSinceInitialized() >= intakeTime) {
    return true;
  }
  return false;
}

void ManipulatorInCmd::End() {
  Robot::manipulatorSub.setIntakePower(0);
}

void ManipulatorInCmd::Interrupted() {
  End();
}
