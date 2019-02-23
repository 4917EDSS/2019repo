#include "commands/ManipulatorOutCmd.h"
#include "Robot.h"

ManipulatorOutCmd::ManipulatorOutCmd() {
  Requires(&Robot::manipulatorSub);
}
void ManipulatorOutCmd::Initialize() {
  Robot::manipulatorSub.setManipulatorWheelSpeed(0.5,0.5);
  Robot::inBallMode = false;
}
void ManipulatorOutCmd::Execute() {}
bool ManipulatorOutCmd::IsFinished() {
  if (TimeSinceInitialized() >= 1.0) {
    return true;
}
  return false;
}
void ManipulatorOutCmd::End() {
  Robot::manipulatorSub.setManipulatorWheelSpeed(0,0);
}
void ManipulatorOutCmd::Interrupted() {
  End();
}
