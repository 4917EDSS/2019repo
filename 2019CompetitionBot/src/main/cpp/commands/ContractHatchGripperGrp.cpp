#include "commands/ContractHatchGripperGrp.h"
#include "commands/HatchGripperContractCmd.h"
#include"commands/ManipulatorInCmd.h"

ContractHatchGripperGrp::ContractHatchGripperGrp() {
  AddSequential(new HatchGripperContractCmd());
  AddSequential(new ManipulatorInCmd(0.25));
}