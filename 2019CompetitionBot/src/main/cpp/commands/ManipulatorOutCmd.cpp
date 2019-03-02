#include "commands/ManipulatorOutCmd.h"
#include "Robot.h"

ManipulatorOutCmd::ManipulatorOutCmd(double intakeTime) : intakeTime(intakeTime) {
  Requires(&Robot::manipulatorSub);
}
ManipulatorOutCmd::ManipulatorOutCmd() {
  Requires(&Robot::manipulatorSub);
  double intakeTime=1.0;
}

void ManipulatorOutCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s\n", __FILE__, __FUNCTION__);
  Robot::manipulatorSub.setIntakePower(0.5);
  Robot::inBallMode = false;
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
