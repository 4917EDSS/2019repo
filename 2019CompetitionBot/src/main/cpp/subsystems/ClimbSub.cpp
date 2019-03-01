/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimbSub.h"
#include "RobotMap.h"

ClimbSub::ClimbSub() : Subsystem("ExampleSubsystem") {
  climbMotor.reset(new rev::CANSparkMax(CLIMB_MOTOR_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
}

void ClimbSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
