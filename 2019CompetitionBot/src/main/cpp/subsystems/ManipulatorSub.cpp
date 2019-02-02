/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ManipulatorSub.h"

ManipulatorSub::ManipulatorSub() : Subsystem("ManipulatorSub") {}

void ManipulatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  hatchGripperSolenoid.reset(new frc::Solenoid(HATCH_GRIPPER_PCM_ID));
  targetDegrees = 90;
  currentState = 0;
  currentDegrees = 90;
}

void ManipulatorSub::ExpandHatchGripper(){
  hatchGripperSolenoid->Set(true);
}

void ManipulatorSub::ContractHatchGripper(){
  hatchGripperSolenoid->Set(false);
}

void ManipulatorSub::IntakeBall(double speed) {
  manipulatorIntakeMotor->Set(speed);
}

bool ManipulatorSub::isBallInManipulator() {
  intakeFromRobotLimit.get();
}

void ManipulatorSub::flipManipulator(bool goForward) {
  if (goForward) {
    targetDegrees = 180;
  } else {
    targetDegrees = 0;
  }
}

bool ManipulatorSub::isManipulatorFlipped() {
  manipulatorFlipperLimit.get();
}

void ManipulatorSub::executeStateMachine() {
  switch (currentState) {
    case 0:
      if (targetDegrees == currentDegrees) {
        //Do nothing
      } else if (targetDegrees < currentDegrees) {
        manipulatorFlipperMotor->Set(-0.5);
        currentState = 1;
      } else {
        manipulatorFlipperMotor->Set(0.5);
        currentState = 1;
      }
      break;
  }
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
