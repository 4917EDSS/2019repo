/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <RobotMap.h>
#include "subsystems/ManipulatorSub.h"
#include "components/Log.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInLayouts.h>
#include "SparkShuffleboardEntrySet.h"

ManipulatorSub::ManipulatorSub() : Subsystem("ManipulatorSub") {
void ManipulatorSub::setManipulatorFlipperMotorSpeed(double speed){
  

  if (getManipulatorEncoder() < -90 && speed < 0){
    speed = 0;
  }

  else if (getManipulatorEncoder() > 90 && speed > 0){
    speed = 0;
  }

   else if (getManipulatorEncoder() > 80 && speed > 0){
    speed = std::min(speed, 0.2);
  }

void ManipulatorSub::setManipulatorTargetAngle(double newAngle) {
  targetAngle = newAngle;
}


  else if (getManipulatorEncoder() < -80 && speed < 0){
    speed = std::max(speed, -0.2);
  }

 double ManipulatorSub::getManipulatorEncoder() {
  return manipulatorFlipperMotor->GetEncoder().GetPosition();
 }

bool ManipulatorSub::isManipulatorAtLimit() {
  return !manipulatorFlipperLimit->Get();
}

void ManipulatorSub::holdManipulatorFlipper(double position){
  //setElevatorMotorSpeed((targetHeight - elevatorMotor1->GetEncoder().GetPosition())* 0.1);
  double holdvalue = (position -  manipulatorFlipperMotor->GetEncoder().GetPosition());
  // logger.send(logger.ELEVATOR, "Flipper motor at target %f, position %f\n", position, manipulatorFlipperMotor->GetEncoder().GetPosition());
  setManipulatorFlipperMotorSpeed(holdvalue*0.1); 
}

double ManipulatorSub::getManipulatorAngle(){
  return  targetAngle;
}

void ManipulatorSub::setManipulatorWheelSpeed(double lspeed, double rspeed) {
  manipulatorIntakeMotorLeft->Set(-lspeed);
  manipulatorIntakeMotorRight->Set(rspeed);
}

bool ManipulatorSub::isBallInManipulator() {
  return !intakeFromRobotLimit->Get();
}

void ManipulatorSub::expandHatchGripper(){
  hatchGripperSolenoid->Set(false);
}

void ManipulatorSub::contractHatchGripper(){
  hatchGripperSolenoid->Set(true);
}


bool ManipulatorSub::isGripperExpanded() {
  return !hatchGripperSolenoid->Get();
}

bool ManipulatorSub::isFinishedMove() {
if (fabs(targetAngle -manipulatorFlipperMotor->GetEncoder().GetPosition()) < MANIPULATOR_POSITION_TOLERANCE && fabs(manipulatorFlipperMotor->GetEncoder().GetVelocity()) < 45) {
      return true;
   } else {
     return false;
   }

void ManipulatorSub::zeroEverything(){
  elevatorMotor1->Set(0.0);
  elevatorMotor2->Set(0.0);
  manipulatorIntakeMotorLeft->Set(0.0);
  manipulatorIntakeMotorRight->Set(0.0);
  manipulatorFlipperMotor->Set(0.0);
}
}

void ManipulatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
