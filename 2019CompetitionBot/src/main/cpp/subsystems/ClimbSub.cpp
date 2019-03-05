/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimbSub.h"
#include "RobotMap.h"

ClimbSub::ClimbSub() : Subsystem("ExampleSubsystem") {
  climbMotor.reset(new WPI_TalonSRX(CLIMB_MOTOR_CAN_ID));
	climbMotor->SetName("Climb");
  climbMotor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);
}

void ClimbSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void ClimbSub::SetClimbMotorSpeed(double speed){
  climbMotor->Set(ControlMode::PercentOutput,-speed);
}

double ClimbSub::getClimbPosition() {
  return climbMotor->GetSensorCollection().GetQuadraturePosition();
}