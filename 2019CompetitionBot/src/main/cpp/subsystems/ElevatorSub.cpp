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
#include "rev/CANSparkMax.h"

constexpr float ELEVATOR_POSITION_TOLERANCE = 5.0;
constexpr float MANIPULATOR_POSITION_TOLERANCE = 1.0;
constexpr float ELEVATOR_P = 0;
constexpr float ELEVATOR_I = 0;
constexpr float ELEVATOR_D = 0;
constexpr float FLIPPER_TICK_TO_DEGREE_FACTOR = (90/44.1900);

ElevatorSub::ElevatorSub() : Subsystem("ElevatorSub") {
  elevatorMotor1.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  elevatorMotor2.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  elevatorMotor1->GetEncoder().SetPosition(0);  // Positive means rotating towards front9
  elevatorMotor2->GetEncoder().SetPosition(0);  // Not used but good to have as a backup

  shifterSolenoid.reset(new frc::Solenoid(CLIMB_GEAR_PCM_ID));
  setShifterHigh(true);

  manipulatorFlipperMotor.reset(new rev::CANSparkMax(MANIPULATOR_FLIPPER_MOTOR_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  manipulatorFlipperMotor->GetEncoder().SetPosition(0);
  manipulatorFlipperMotor->GetEncoder().SetPositionConversionFactor(FLIPPER_TICK_TO_DEGREE_FACTOR);
  intakeFromRobotLimit.reset(new frc::DigitalInput(BALL_SENSOR_DIO));

  manipulatorIntakeMotorLeft.reset(new WPI_VictorSPX(MANIPULATOR_LEFT_INTAKE_MOTOR_CAN_ID));
  manipulatorIntakeMotorRight.reset(new WPI_VictorSPX(MANIPULATOR_RIGHT_INTAKE_MOTOR_CAN_ID));

  hatchGripperSolenoid.reset(new frc::Solenoid(HATCH_GRIPPER_PCM_ID));
  expandHatchGripper();
}

void ElevatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new ElevatorWithJoystickCmd()); 
}

void ElevatorSub::update(){
  //setElevatorMotorSpeed((targetHeight - elevatorMotor1->GetEncoder().GetPosition())* 0.1);
  setManipulatorFlipperMotorSpeed((targetDegrees -  manipulatorFlipperMotor->GetEncoder().GetPosition())* 0.04);
}

// Set true for High Gear, false for Low Gear
void ElevatorSub::setShifterHigh(bool highGear){
  shifterSolenoid->Set(highGear);
}

bool ElevatorSub::isShifterHigh(){
  return shifterSolenoid->Get();
}

void ElevatorSub::expandHatchGripper(){
  hatchGripperSolenoid->Set(false);
}

void ElevatorSub::contractHatchGripper(){
  hatchGripperSolenoid->Set(true);
}

bool ElevatorSub::isGripperExpanded() {
  return !hatchGripperSolenoid->Get();
}

void ElevatorSub::zeroEverything(){
  elevatorMotor1->Set(0.0);
  elevatorMotor2->Set(0.0);
  manipulatorIntakeMotorLeft->Set(0.0);
  manipulatorIntakeMotorRight->Set(0.0);
  manipulatorFlipperMotor->Set(0.0);
}

// Positive speed is out, negative is in
void ElevatorSub::setManipulatorWheelSpeed(double lspeed, double rspeed) {
  manipulatorIntakeMotorLeft->Set(-lspeed);
  manipulatorIntakeMotorRight->Set(rspeed);
}

bool ElevatorSub::isBallInManipulator() {
  return !intakeFromRobotLimit->Get();
}

 double ElevatorSub::getElevatorEncoder() {
  return elevatorMotor1->GetEncoder().GetPosition();
 }

 double ElevatorSub::getManipulatorEncoder() {
  return manipulatorFlipperMotor->GetEncoder().GetPosition();
 }

bool ElevatorSub::isManipulatorAtLimit() {
  return !manipulatorFlipperLimit->Get();
}

void ElevatorSub::setElevatorTargetHeight(double newHeight){
  targetHeight = newHeight;
}

void ElevatorSub::setManipulatorTargetAngle(double newAngle) {
  targetDegrees = newAngle;
}

bool ElevatorSub::isFinishedMove() {
 if (fabs(targetHeight -elevatorMotor1->GetEncoder().GetPosition()) < ELEVATOR_POSITION_TOLERANCE && fabs(elevatorMotor1->GetEncoder().GetVelocity()) < 45) {
   if (fabs(targetDegrees -manipulatorFlipperMotor->GetEncoder().GetPosition()) < MANIPULATOR_POSITION_TOLERANCE && fabs(manipulatorFlipperMotor->GetEncoder().GetVelocity()) < 45) {
      return true;
   } else {
     return false;
   }
 } else{
   return false;
 }
}

bool ElevatorSub::isElevatorDown(){
  return !lowerLimit.get() && elevatorMotor1->GetEncoder().GetPosition();
}

void ElevatorSub::setElevatorMotorRaw(double speed){
  elevatorMotor1->Set(-speed);
  elevatorMotor2->Set(speed);
}

void ElevatorSub::setManipulatorFlipperMotorSpeed(double speed){
  

  if (getManipulatorEncoder() < -90 && speed < 0){
    speed = 0;
  }

  else if (getManipulatorEncoder() > 90 && speed > 0){
    speed = 0;
  }

   else if (getManipulatorEncoder() > 80 && speed > 0){
    speed = std::min(speed, 0.2);
  }


  else if (getManipulatorEncoder() < -80 && speed < 0){
    speed = std::max(speed, -0.2);
  }

  manipulatorFlipperMotor->Set(speed);

} 
void ElevatorSub::setElevatorMotorSpeed(double speed){

  if (isElevatorDown() && speed < 0){
        speed = 0;
  }

   else if (elevatorMotor1->GetEncoder().GetPosition() > 150 && speed > 0){
    speed = 0;
  }

  else if (elevatorMotor1->GetEncoder().GetPosition() < 20 && speed < 0){
    speed = std::max(speed, -0.2);
  }


  else if (elevatorMotor1->GetEncoder().GetPosition() > 100 && speed > 0){
    speed = std::min(speed, 0.2);
  }
}
