#include "commands/ManipulatorInCmd.h"
#include "Robot.h"
ManipulatorInCmd::ManipulatorInCmd () {
  Requires(&Robot::manipulatorSub);
}
void ManipulatorInCmd::Initialize() {
  Robot::manipulatorSub.setManipulatorWheelSpeed(-0.5,-0.5);
    Robot::inBallMode = true;
}
void ManipulatorInCmd::Execute() {}
bool ManipulatorInCmd::IsFinished() {
  if (TimeSinceInitialized() >= 1.0) {
    return true;
}
  return false;
}
void ManipulatorInCmd::End() {
  Robot::manipulatorSub.setManipulatorWheelSpeed(0,0);
}
void ManipulatorInCmd::Interrupted() {
  End();
}
