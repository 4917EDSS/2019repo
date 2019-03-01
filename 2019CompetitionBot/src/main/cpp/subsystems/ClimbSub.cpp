/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimbSub.h"
#include "RobotMap.h"

ClimbSub::ClimbSub() : Subsystem("ExampleSubsystem") {
climbMotor.reset(new WPI_VictorSPX(CLIMB_MOTOR_CAN_ID));
	climbMotor->SetName("Climb");
}

void ClimbSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void ClimbSub::SetClimbMotor(double speed){
  climbMotor->Set(speed);
}
