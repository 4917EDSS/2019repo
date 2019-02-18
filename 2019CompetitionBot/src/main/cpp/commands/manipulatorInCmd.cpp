#include "commands/manipulatorInCmd.h"
#include "Robot.h"
manipulatorInCmd::manipulatorInCmd () {
  Requires(&Robot::elevatorSub);
}
void manipulatorInCmd::Initialize() {
  Robot::elevatorSub.setManipulatorWheelSpeed(-0.5,-0.5);
}
void manipulatorInCmd::Execute() {}
bool manipulatorInCmd::IsFinished() {
  if (TimeSinceInitialized() >= 1.0) {
    return true;
}
  return false;
}
void manipulatorInCmd::End() {
  Robot::elevatorSub.setManipulatorWheelSpeed(0,0);
}
void manipulatorInCmd::Interrupted() {
  End();
}