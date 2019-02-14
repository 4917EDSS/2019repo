/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <RobotMap.h>
#include "subsystems/ElevatorSub.h"
#include "commands/ElevatorWithJoystickCmd.h"
#include <ctre/Phoenix.h>
#include "components/Log.h"
#include "frc/WPILib.h"

constexpr float ELEVATOR_POSITION_TOLERANCE = 5.0;
constexpr float ELEVATOR_P = 0;
constexpr float ELEVATOR_I = 0;
constexpr float ELEVATOR_D = 0;

ElevatorSub::ElevatorSub() : Subsystem("ExampleSubsystem") {
  elevatorMotor.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
	logger.send(logger.ELEVATOR, "Elevator code started \n", 0.0);

  hatchGripperSolenoid.reset(new frc::Solenoid(HATCH_GRIPPER_PCM_ID));
  manipulatorIntakeMotorLeft.reset(new WPI_VictorSPX(MANIPULATOR_LEFT_INTAKE_MOTOR_CAN_ID));
  manipulatorIntakeMotorRight.reset(new WPI_VictorSPX(MANIPULATOR_RIGHT_INTAKE_MOTOR_CAN_ID));
  manipulatorFlipperMotor.reset(new rev::CANSparkMax(MANIPULATOR_FLIPPER_MOTOR_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
}

void ElevatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new ElevatorWithJoystickCmd()); 
}

void ElevatorSub::update(){
  setElevatorMotor((targetHeight - elevatorMotor->GetEncoder().GetPosition())* 0.1);
}

void ElevatorSub::ExpandHatchGripper(){
  hatchGripperSolenoid->Set(true);
}

void ElevatorSub::ContractHatchGripper(){
    hatchGripperSolenoid->Set(false);
}

void ElevatorSub::zeroEverything(){
  elevatorMotor->Set(0.0);
  manipulatorIntakeMotorLeft->Set(0.0);
  manipulatorIntakeMotorRight->Set(0.0);
  manipulatorFlipperMotor->Set(0.0);
}

void ElevatorSub::setWheels(double lspeed, double rspeed) {
  manipulatorIntakeMotorLeft->Set(lspeed);
  manipulatorIntakeMotorRight->Set(rspeed);
}

bool ElevatorSub::isBallInManipulator() {
  intakeFromRobotLimit.get();
}

 double ElevatorSub::getManipulatorEncoder() {
  return manipulatorFlipperMotor->GetEncoder().GetPosition();
 }

void ElevatorSub::flipManipulator(bool goForward) {
  if (goForward) {
    targetDegrees = 180;
  } else {
    targetDegrees = 0;
  }
}

bool ElevatorSub::isManipulatorFlipped() {
  manipulatorFlipperLimit.get();
}

void ElevatorSub::executeStateMachine() {
  double currentDegrees = getManipulatorEncoder();

  if (fabs(targetDegrees - currentDegrees) <= 2.5) {
    //Do nothing
  } else if (targetDegrees < currentDegrees) {
    manipulatorFlipperMotor->Set(-0.5);
  } else {
    manipulatorFlipperMotor->Set(0.5);
  }
}

void ElevatorSub::setTargetHeight(double newHeight){
  targetHeight = newHeight;
}

void ElevatorSub::setTargetAngle(double newAngle) {
  targetDegrees = newAngle;
}

bool ElevatorSub::isFinishedMove() {
 if (fabs(targetHeight -elevatorMotor->GetEncoder().GetPosition()) < ELEVATOR_POSITION_TOLERANCE && fabs(elevatorMotor->GetEncoder().GetPosition()) < 45) {
  return true;
    } else {
      return false;
 }
}

bool ElevatorSub::isElevatorDown(){
      return !lowerLimit.get() && elevatorMotor->GetEncoder().GetPosition();
}

void ElevatorSub::setElevatorMotorRaw(double speed){
  elevatorMotor->Set(speed);
}

void ElevatorSub::setManipulatorFlipperMotor(double speed){
  manipulatorFlipperMotor->Set(speed);

}
void ElevatorSub::setElevatorMotor(double speed){

  if (isElevatorDown() && speed < 0){
        speed = 0;
  }

  else if (elevatorMotor->GetEncoder().GetPosition() < 20 && speed <0){
    speed = std::max(speed, -0.2);
  }

  else if (250 && speed < 0){
    speed = 0;
  }
  //Change 250 to max elevator height.

  else if (elevatorMotor->GetEncoder().GetPosition() > 100 && speed > 0){
    speed = std::min(speed, 0.2);
  }
}



// Put methods for controlling this subsystem
// here. Call these from Commands.
