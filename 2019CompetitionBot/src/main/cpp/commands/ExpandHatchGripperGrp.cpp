#include "commands/ExpandHatchGripperGrp.h"
#include "commands/HatchGripperExpandCmd.h"
#include "commands/ManipulatorInCmd.h"

ExpandHatchGripperGrp::ExpandHatchGripperGrp() {
  AddSequential(new HatchGripperExpandCmd());
  AddSequential( new ManipulatorInCmd(0.4));
}