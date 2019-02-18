#include "commands/manipulatorOutCmd.h"
#include "Robot.h"
manipulatorOutCmd::manipulatorOutCmd() {
  Requires(&Robot::elevatorSub);
}
void manipulatorOutCmd::Initialize() {
  Robot::elevatorSub.setManipulatorWheelSpeed(0.5,0.5);
}
void manipulatorOutCmd::Execute() {}
bool manipulatorOutCmd::IsFinished() {
  if (TimeSinceInitialized() >= 1.0) {
    return true;
}
  return false;
}
void manipulatorOutCmd::End() {
  Robot::elevatorSub.setManipulatorWheelSpeed(0,0);
}
void manipulatorOutCmd::Interrupted() {
  End();
}
