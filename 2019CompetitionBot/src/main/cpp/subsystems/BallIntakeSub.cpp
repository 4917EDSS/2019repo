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
#include "SparkShuffleboardEntrySet.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "frc/shuffleboard/BuiltInLayouts.h"
//Canid4 and Set Intake
#define ENCODER_SCALE (90.0 / 32.0)

BallIntakeSub::BallIntakeSub() : Subsystem("BallIntakeSub")
{
  ballIntakeMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(BALL_INTAKE_WHEELS_MOTOR_CAN_ID));
  currentSpeed = 0;
  flipperMotorOne.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(BALL_INTAKE_TOP_FLIP_MOTOR_1_CAN_ID));
  flipperMotorTwo.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(BALL_INTAKE_BOTTOM_FLIP_MOTOR_2_CAN_ID));
  intakeFolderSolenoid.reset(new frc::Solenoid(BALL_INTAKE_FOLDER_PCM_ID));
  ballIntakeArmLimit.reset(new frc::DigitalInput(BALL_INTAKE_ARM_LIMIT_DIO));
  setFolderOut(true);
  intakeArmEnc.reset(new frc::Encoder(INTAKE_MOTOR_ENC1_DIO, INTAKE_MOTOR_ENC2_DIO));

  frc::ShuffleboardTab& shuffleTab = frc::Shuffleboard::GetTab("Ball Intake");

  nteBallIntakeMotor = (shuffleTab.Add("Intake", 0).GetEntry()); 
  nteFlipperMotorOne = (shuffleTab.Add("Flipper 1", 0).GetEntry());
  nteFlipperMotorOne = (shuffleTab.Add("Flipper 2", 0).GetEntry());
  nteIntakeFolderSolenoid = (shuffleTab.Add("Folder", 0).GetEntry());
  nteBallIntakeArmLimit = (shuffleTab.Add("Intake Limit", 0).GetEntry());
  nteIntakeArmEncPosition = (shuffleTab.Add("Intake Position", 0).GetEntry());
}

void BallIntakeSub::updateShuffleBoard(){
  nteBallIntakeMotor.SetDouble(ballIntakeMotor->Get());
  nteFlipperMotorOne.SetDouble(flipperMotorOne->Get());
  nteFlipperMotorTwo.SetDouble(flipperMotorTwo->Get());
  nteIntakeFolderSolenoid.SetBoolean(intakeFolderSolenoid->Get());
  nteBallIntakeArmLimit.SetBoolean(ballIntakeArmLimit->Get());
  nteIntakeArmEncPosition.SetDouble(intakeArmEnc->Get());
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

bool BallIntakeSub::getIntakeArmLimit(){
return !ballIntakeArmLimit->Get();
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
  double pValueFlipOut = 0.003;
  double nValueFlipOut = 0.01; //apply higher power if the motors are fighting gravity
  double pValueFlipIn = 0.003;
  double nValueFlipIn = 0.01;
  double speed = 0; //will be changed
  bool flipPosition = isFolderOut();
  std::cout << "The flipper is out: " << flipPosition << std::endl;
  std::cout << "The limit is hit: " << getIntakeArmLimit() << std::endl;
  if (isClimbing)
  {
    speed = (targetAngle - currentAngle) * 0.01;
  }
  else
  {
    speed = (targetAngle - currentAngle);
  }

  if (speed < 0)
  {
    if(flipPosition){
    speed = speed * nValueFlipOut;
    } else {
    speed = speed * nValueFlipIn;
    }
  }
  else
  {
    if(flipPosition){
    speed = speed * pValueFlipOut;
    } else {
    speed = speed * pValueFlipIn;
    }
  }
  //std::cout << "Speed: " << speed << " target angle " << targetAngle << " Current angle: " << currentAngle << "\n";
    //speed = std::min(speed, 0.35);
    //speed = std::max(speed, -0.35);
if(speed > 0 && currentAngle > 150){
  speed = 0;
}
if(speed < 0 && getIntakeArmLimit()){
  speed = 0;
}

  if (!(fabs(targetAngle - currentAngle) < 2))
  {
    setIntakeArmMotor(speed, isClimbing);
  }
}

bool BallIntakeSub::doneFlipping()
{
  return false;
}
