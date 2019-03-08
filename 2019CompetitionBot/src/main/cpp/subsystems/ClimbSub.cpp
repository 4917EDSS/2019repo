
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/ClimbSub.h"
#include "RobotMap.h"
#include <frc/shuffleboard/Shuffleboard.h>

ClimbSub::ClimbSub() : Subsystem("ExampleSubsystem") {
  climbMotor.reset(new WPI_TalonSRX(CLIMB_MOTOR_CAN_ID));
	climbMotor->SetName("Climb");
  climbMotor->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder);

  frc::ShuffleboardTab &shuffleTab = frc::Shuffleboard::GetTab("Climb");

  ntePower = (shuffleTab.Add("Power", 0).GetEntry());
  ntePosition = (shuffleTab.Add("Position", 0).GetEntry());
}

void ClimbSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void ClimbSub::updateShuffleBoard() {
  ntePower.SetDouble(climbMotor->Get());
  ntePosition.SetDouble(getClimbPosition());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
int ClimbSub::GetClimbEncoder() {
  return climbMotor->GetSensorCollection().GetQuadraturePosition();
}
void ClimbSub::SetClimbMotorPower(double power){
  climbMotor->Set(ControlMode::PercentOutput, -power);
}

double ClimbSub::getClimbPosition() {
  // TODO:  Return in units of mm and change constants in h file to match
  return climbMotor->GetSensorCollection().GetQuadraturePosition();
}