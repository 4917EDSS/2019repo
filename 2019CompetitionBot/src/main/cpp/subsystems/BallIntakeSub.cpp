/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/BallIntakeSub.h"
#include <ctre/Phoenix.h>
#include "components/Log.h"
#include <iostream>
//Canid4 and Set Intake
#define ENCODER_SCALE (90.0/32.0)

BallIntakeSub::BallIntakeSub() : Subsystem("ExampleSubsystem") {
  ballIntakeMotor.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(BALL_INTAKE_WHEELS_MOTOR_CAN_ID));
  flipperMotorOne.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(BALL_INTAKE_TOP_FLIP_MOTOR_1_CAN_ID));
  flipperMotorTwo.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(BALL_INTAKE_BOTTOM_FLIP_MOTOR_2_CAN_ID));
  intakeFolderSolenoid.reset(new frc::Solenoid(BALL_INTAKE_FOLDER_PCM_ID));
  setFlipperOut(true);
  intakeArmEnc.reset(new frc::Encoder(INTAKE_MOTOR_ENC1_DIO, INTAKE_MOTOR_ENC2_DIO));
}

void BallIntakeSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
/*
needs to work on the SetIntakeArmAngle
void BallIntakeSub::setIn(){
  flipperMotorOne.SetAngle(0.0);
  flipperMotorTwo.SetAngle(0.0);
}

void BallIntakeSub::setOut(){
  flipperMotorOne.SetAngle(90.0);
  flipperMotorTwo.SetAngle(90.0);
}
*/
void BallIntakeSub::setArmTargetPosition(double angle){
  targetAngle = angle;
}

void BallIntakeSub::setIntakeMotor(double speed){
  ballIntakeMotor->Set(ControlMode::PercentOutput, speed);
  logger.send(logger.DEBUGGING, "%s\n", __FUNCTION__);
}

double BallIntakeSub::getIntakeArmEncoderAngle() {
  //This assumes 5 encoder ticks per degree, this will need to be tested
  
  return -(intakeArmEnc->GetDistance() * ENCODER_SCALE);
}

void BallIntakeSub::setFlipperOut(bool flipOut) {
  intakeFolderSolenoid->Set(!flipOut);
 
  logger.send(logger.DEBUGGING, "%s\n", __FUNCTION__);
}

void BallIntakeSub::setIntakeArmMotor(double speed){
  flipperMotorOne->Set(ControlMode::PercentOutput, -speed);
  flipperMotorTwo->Set(ControlMode::PercentOutput, -speed);
}

void BallIntakeSub::update(bool isClimbing){  
  double currentAngle = getIntakeArmEncoderAngle();
  double pValue =  0.015;
 
  double speed = (targetAngle - currentAngle) * pValue;
  std::cout << "Speed: " << speed << " target angle " << targetAngle << " Current angle: " << currentAngle << "\n";
  //speed += (53 - currentAngle) * 0.0001;
  /*
  if (!isClimbing) {
    speed = std::min(speed, 0.35);
    speed = std::max(speed, -0.35);
  }
*/
  //setIntakeArmMotor(speed);
}

bool BallIntakeSub::doneFlipping() {
  if (((fabs(targetAngle - getIntakeArmEncoderAngle())) < 2) && (fabs(intakeArmEnc->GetRate()) < 5))  {
    return true;
  }  else {
    return false;
  }
}