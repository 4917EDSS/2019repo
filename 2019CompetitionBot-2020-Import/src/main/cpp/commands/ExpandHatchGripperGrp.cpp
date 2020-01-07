#include "commands/ExpandHatchGripperGrp.h"
#include "commands/HatchGripperExpandCmd.h"
#include "commands/ManipulatorInCmd.h"
#include "commands/DoNothingHatchCmd.h"

ExpandHatchGripperGrp::ExpandHatchGripperGrp() {
  AddParallel(new DoNothingHatchCmd());
  AddSequential(new HatchGripperExpandCmd());
  AddSequential( new ManipulatorInCmd(0.6));
}