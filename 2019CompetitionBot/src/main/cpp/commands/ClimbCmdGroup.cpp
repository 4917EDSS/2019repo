#include "commands/ClimbCmdGroup.h"
#include "commands/SetIntakeArmAngleCmd.h"
#include "commands/DriveStraightCmd.h"
#include "commands/DriveWithIntakeMotorCmd.h"

ClimbCmdGroup::ClimbCmdGroup() {
  AddSequential(new SetIntakeArmAngleCmd(true, 90));
  AddParallel(new DriveStraightCmd(1000000000.0));
  AddSequential(new DriveWithIntakeMotorCmd());
}
