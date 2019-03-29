#include "commands/ManipulatorInCmd.h"
#include "Robot.h"
ManipulatorInCmd::ManipulatorInCmd (double intakeTime) : intakeTime(intakeTime) {
  Requires(&Robot::manipulatorSub);
}
ManipulatorInCmd::ManipulatorInCmd() : intakeTime(1.0) {
  Requires(&Robot::manipulatorSub);
}

void ManipulatorInCmd::Initialize() {
  //logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::manipulatorSub.setIntakePower(-0.5);
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
