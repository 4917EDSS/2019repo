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
#include "commands/BallIntakeWithJoystickCmd.h"
//Canid4 and Set Intake
#define ENCODER_SCALE (90.0 / 32.0)

BallIntakeSub::BallIntakeSub() : Subsystem("BallIntakeSub")
{
  ballIntakeMotor.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(BALL_INTAKE_WHEELS_MOTOR_CAN_ID));
  currentSpeed = 0;
  flipperMotorOne.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(BALL_INTAKE_TOP_FLIP_MOTOR_1_CAN_ID));
  flipperMotorTwo.reset(new ctre::phoenix::motorcontrol::can::VictorSPX(BALL_INTAKE_BOTTOM_FLIP_MOTOR_2_CAN_ID));
  intakeFolderSolenoid.reset(new frc::Solenoid(BALL_INTAKE_FOLDER_PCM_ID));
  ballIntakeArmLimit.reset(new frc::DigitalInput(BALL_INTAKE_ARM_LIMIT_DIO));
  setFolderOut(true);
  intakeArmEnc.reset(new frc::Encoder(INTAKE_MOTOR_ENC1_DIO, INTAKE_MOTOR_ENC2_DIO));
}

void BallIntakeSub::InitDefaultCommand()
{
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new BallIntakeWithJoystickCmd());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void BallIntakeSub::setArmTargetPosition(double angle)
{
  targetAngle = angle;
}

void BallIntakeSub::setIntakeWheelsMotorSpeed(double speed)
{
  ballIntakeMotor->Set(ControlMode::PercentOutput, speed);
  currentSpeed = speed;
  logger.send(logger.DEBUGGING, "%s\n", __FUNCTION__);
}

double BallIntakeSub::getIntakeWheelsMotorSpeed()
{
  return currentSpeed;
}

double BallIntakeSub::getIntakeArmEncoderAngle()
{
  //This assumes 5 encoder ticks per degree, this will need to be tested

  return -(intakeArmEnc->GetDistance() * ENCODER_SCALE);
}

void BallIntakeSub::setFolderOut(bool flipOut)
{
  intakeFolderSolenoid->Set(!flipOut);

  logger.send(logger.DEBUGGING, "%s\n", __FUNCTION__);
}

bool BallIntakeSub::isFolderOut()
{
  return !intakeFolderSolenoid->Get();
}

void BallIntakeSub::setIntakeArmMotor(double speed, bool isClimbing)
{
  flipperMotorOne->Set(ControlMode::PercentOutput, -speed);
  if (isClimbing)
  {
    flipperMotorTwo->Set(ControlMode::PercentOutput, -speed);
  }
}

void BallIntakeSub::update(bool isClimbing)
{
  double currentAngle = getIntakeArmEncoderAngle();
  double pValue = 0.003;
  double nValue = 0.01; //apply higher power if the motors are fighting gravity
  double speed = 0; //will be changed
  if (isClimbing)
  {
    speed = (targetAngle - currentAngle) * pValue * 0.001;
  }
  else
  {
    speed = (targetAngle - currentAngle);
  }

  if (speed < 0)
  {
    speed = speed * nValue;
  }
  else
  {
    speed = speed * pValue;
  }
  std::cout << "Speed: " << speed << " target angle " << targetAngle << " Current angle: " << currentAngle << "\n";
  //speed += (53 - currentAngle) * 0.0001;
  /*
  if (!isClimbing) {
    speed = std::min(speed, 0.35);
    speed = std::max(speed, -0.35);
  }
*/

  if (!(fabs(targetAngle - currentAngle) < 2))
  {
    keepArmAtTarget(speed, isClimbing);
  }
}

void BallIntakeSub::keepArmAtTarget(double speed, bool isClimbing)
{
  setIntakeArmMotor(speed, isClimbing);
}

bool BallIntakeSub::doneFlipping()
{

  /*
  if (((fabs(targetAngle - getIntakeArmEncoderAngle())) < 2) && (fabs(intakeArmEnc->GetRate()) < 5))  {
    return true;
  }  else {
    return false;
  }
  */
  return false;
}
