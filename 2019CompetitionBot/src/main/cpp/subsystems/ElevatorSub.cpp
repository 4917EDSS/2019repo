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
  elevatorMotor.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
	logger.send(logger.ELEVATOR, "Elevator code started @ %f\n", 0.0);
}

void ElevatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new ElevatorWithJoystickCmd());
   
}

void ElevatorSub::update(){
  setElevatorMotor((target - elevatorMotor->GetEncoder().GetPosition())* 0.1);
}

void ElevatorSub::setTarget(double newTarget){
  target = newTarget;
}

bool ElevatorSub::isFinishedMove(){
 if(fabs(target -elevatorMotor->GetEncoder().GetPosition()) < ELEVATOR_POSITION_TOLERANCE && fabs(elevatorMotor->GetEncoder().GetPosition()) < 45) {
  return true;
    }else{
      return false;
 }
}

bool ElevatorSub::isElevatorDown(){
      return !lowerLimit.get() && elevatorMotor->GetEncoder().GetPosition();
}

void ElevatorSub::setElevatorMotorRaw(double speed){
elevatorMotor->Set(speed);
}

void ElevatorSub::setElevatorMotor(double speed){

  if (isElevatorDown() && speed < 0){
        speed = 0;
  }

  else if (elevatorMotor->GetEncoder().GetPosition() < 20 && speed <0){
    speed = std::max(speed, -0.2);
  }
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
