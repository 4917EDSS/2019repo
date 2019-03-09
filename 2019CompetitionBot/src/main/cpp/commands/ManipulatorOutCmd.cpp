#include "commands/ManipulatorOutCmd.h"
#include "Robot.h"

ManipulatorOutCmd::ManipulatorOutCmd(double intakeTime) : intakeTime(intakeTime) {
  Requires(&Robot::manipulatorSub);
}
ManipulatorOutCmd::ManipulatorOutCmd() : intakeTime(1.0) {
  Requires(&Robot::manipulatorSub);
}

void ManipulatorOutCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::manipulatorSub.setIntakePower(0.5);
}

void ManipulatorOutCmd::Execute() {}

bool ManipulatorOutCmd::IsFinished() {
  if (TimeSinceInitialized() >= intakeTime) {
    return true;
  }
  return false;
}

void ManipulatorOutCmd::End() {
  Robot::manipulatorSub.setIntakePower(0);
}

void ManipulatorOutCmd::Interrupted() {
  End();
}
