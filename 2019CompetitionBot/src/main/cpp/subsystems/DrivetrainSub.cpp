/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/DrivetrainSub.h"
#include "commands/DriveWithJoystickCmd.h"
#include <RobotMap.h>
#include <iostream>
#include <hal/HAL.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include "frc/shuffleboard/BuiltInLayouts.h"

constexpr float DRIVE_BALANCE_TOLERANCE = 0.5;
constexpr float DRIVE_BALANCE_P = 0;
constexpr float DRIVE_BALANCE_I = 0;
constexpr float DRIVE_BALANCE_D = 0;
constexpr float MOTOR_POWER_SCALING_FACTOR = 1.0;  // TODO:  Is this necessary?

void DrivetrainSub::SetDrivetrainEncoderZero(){
  rightMotor1->GetEncoder().SetPosition(0);
  rightMotor2->GetEncoder().SetPosition(0);
  rightMotor3->GetEncoder().SetPosition(0);

  leftMotor1->GetEncoder().SetPosition(0);
  leftMotor2->GetEncoder().SetPosition(0);
  leftMotor3->GetEncoder().SetPosition(0);
}

DrivetrainSub::DrivetrainSub() : Subsystem("DrivetrainSub"){

  rightMotor1.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor2.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor3.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_3_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

  leftMotor1.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor2.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor3.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_3_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

  rightMotor1->GetEncoder().SetPosition(0);
  rightMotor2->GetEncoder().SetPosition(0);
  rightMotor3->GetEncoder().SetPosition(0);
  
  leftMotor1->GetEncoder().SetPosition(0);
  leftMotor2->GetEncoder().SetPosition(0);
  leftMotor3->GetEncoder().SetPosition(0);

  rightMotor1->SetSmartCurrentLimit(50);
  rightMotor2->SetSmartCurrentLimit(50);
  rightMotor3->SetSmartCurrentLimit(50);

  leftMotor1->SetSmartCurrentLimit(50);
  leftMotor2->SetSmartCurrentLimit(50);
  leftMotor3->SetSmartCurrentLimit(50);

  ahrs.reset(new AHRS(frc::SPI::kMXP));
  ahrs->SetName("Drivetrain", "AHRS");

  driveBalancer.reset(new frc4917::MotorBalancer());
  driveBalancePID.reset(new frc::PIDController(DRIVE_BALANCE_P, DRIVE_BALANCE_I, DRIVE_BALANCE_D, ahrs.get(), driveBalancer.get()));

  // Post all sensors and actuators to the Shuffleboard
  frc::ShuffleboardTab& shuffleTab = frc::Shuffleboard::GetTab("Drivetrain");

  for(int motorIndex = 0; motorIndex < 6; motorIndex++)
  {
    std::string listName = "Motor " + std::to_string(motorIndex) + " Data";
    frc::ShuffleboardLayout& shuffleList = shuffleTab.GetLayout(listName, frc::BuiltInLayouts::kList);
    shuffleList.WithSize(1,3);
    shuffleList.WithPosition(motorIndex,0);

    nteSparks[motorIndex].setPower = (shuffleList.Add("Set Power", 0).GetEntry());
    nteSparks[motorIndex].outputCurrent = (shuffleList.Add("Current Out", 0).GetEntry());
    nteSparks[motorIndex].encoderPosition = (shuffleList.Add("Position", 0).GetEntry());
    nteSparks[motorIndex].encoderVelocity = (shuffleList.Add("Velocity", 0).GetEntry());
    nteSparks[motorIndex].motorTemperature = (shuffleList.Add("Motor Temp", 0).GetEntry());
  }

  frc::ShuffleboardLayout& shuffleList = shuffleTab.GetLayout("AHRS", frc::BuiltInLayouts::kList);
  shuffleList.WithSize(1,2);
  shuffleList.WithPosition(6,0);
  nteYaw = (shuffleList.Add("Yaw", 0).GetEntry());
  ntePitch = (shuffleList.Add("Pitch", 0).GetEntry());
  nteRoll = (shuffleList.Add("Roll", 0).GetEntry());
  
}

double DrivetrainSub::getRightEncoder()
{
  return -(rightMotor1->GetEncoder().GetPosition());
}

double DrivetrainSub::getLeftEncoder()
{
  return leftMotor1->GetEncoder().GetPosition();
}

void DrivetrainSub::InitDefaultCommand()
{
  // Set the default command for a subsystem here.
  SetDefaultCommand(new DriveWithJoystickCmd());
}

void DrivetrainSub::drive(double lSpeed, double rSpeed)
{
  leftMotor1->Set(-lSpeed * MOTOR_POWER_SCALING_FACTOR);
  leftMotor2->Set(-lSpeed * MOTOR_POWER_SCALING_FACTOR);
  leftMotor3->Set(-lSpeed * MOTOR_POWER_SCALING_FACTOR);

  rightMotor1->Set(rSpeed * MOTOR_POWER_SCALING_FACTOR);
  rightMotor2->Set(rSpeed * MOTOR_POWER_SCALING_FACTOR);
  rightMotor3->Set(rSpeed * MOTOR_POWER_SCALING_FACTOR);
}

void DrivetrainSub::updateShuffleBoard(){
  nteSparks[0].setPower.SetDouble(leftMotor1->Get());
  nteSparks[0].outputCurrent.SetDouble(leftMotor1->GetOutputCurrent());
  nteSparks[0].encoderPosition.SetDouble(leftMotor1->GetEncoder().GetPosition());
  nteSparks[0].encoderVelocity.SetDouble(leftMotor1->GetEncoder().GetVelocity());
  nteSparks[0].motorTemperature.SetDouble(leftMotor1->GetMotorTemperature());

  nteSparks[1].setPower.SetDouble(leftMotor2->Get());
  nteSparks[1].outputCurrent.SetDouble(leftMotor2->GetOutputCurrent());
  nteSparks[1].encoderPosition.SetDouble(leftMotor2->GetEncoder().GetPosition());
  nteSparks[1].encoderVelocity.SetDouble(leftMotor2->GetEncoder().GetVelocity());
  nteSparks[1].motorTemperature.SetDouble(leftMotor2->GetMotorTemperature());

  nteSparks[2].setPower.SetDouble(leftMotor3->Get());
  nteSparks[2].outputCurrent.SetDouble(leftMotor3->GetOutputCurrent());
  nteSparks[2].encoderPosition.SetDouble(leftMotor3->GetEncoder().GetPosition());
  nteSparks[2].encoderVelocity.SetDouble(leftMotor3->GetEncoder().GetVelocity());
  nteSparks[2].motorTemperature.SetDouble(leftMotor3->GetMotorTemperature());

  nteSparks[3].setPower.SetDouble(rightMotor1->Get());
  nteSparks[3].outputCurrent.SetDouble(rightMotor1->GetOutputCurrent());
  nteSparks[3].encoderPosition.SetDouble(rightMotor1->GetEncoder().GetPosition());
  nteSparks[3].encoderVelocity.SetDouble(rightMotor1->GetEncoder().GetVelocity());
  nteSparks[3].motorTemperature.SetDouble(rightMotor1->GetMotorTemperature());

  nteSparks[4].setPower.SetDouble(rightMotor2->Get());
  nteSparks[4].outputCurrent.SetDouble(rightMotor2->GetOutputCurrent());
  nteSparks[4].encoderPosition.SetDouble(rightMotor2->GetEncoder().GetPosition());
  nteSparks[4].encoderVelocity.SetDouble(rightMotor2->GetEncoder().GetVelocity());
  nteSparks[4].motorTemperature.SetDouble(rightMotor2->GetMotorTemperature());

  nteSparks[5].setPower.SetDouble(rightMotor1->Get());
  nteSparks[5].outputCurrent.SetDouble(rightMotor3->GetOutputCurrent());
  nteSparks[5].encoderPosition.SetDouble(rightMotor3->GetEncoder().GetPosition());
  nteSparks[5].encoderVelocity.SetDouble(rightMotor3->GetEncoder().GetVelocity());
  nteSparks[5].motorTemperature.SetDouble(rightMotor3->GetMotorTemperature());

  ntePitch.SetDouble(ahrs->GetPitch());
  nteYaw.SetDouble(getAngle());
  nteRoll.SetDouble(ahrs->GetRoll());
}
void DrivetrainSub::enableBalancerPID(float setPoint){
  Preferences *prefs = Preferences::GetInstance();
	driveBalancePID->SetPID(prefs->GetFloat("DriveBalanceP", DRIVE_BALANCE_P), prefs->GetFloat("DriveBalanceI", DRIVE_BALANCE_I), prefs->GetFloat("DriveBalanceD", DRIVE_BALANCE_D));
	driveBalancePID->SetAbsoluteTolerance(prefs->GetFloat("DriveBalanceTolerance", DRIVE_BALANCE_TOLERANCE));
	driveBalancePID->SetSetpoint(setPoint);
	driveBalancer->Reset();
	driveBalancePID->Enable();
}

void DrivetrainSub::disableBalancerPID(){
	driveBalancer->Reset();
	driveBalancePID->Disable();
}
void DrivetrainSub::driverDriveStraight(float speed) {
	drive(speed + driveBalancer->GetDifference(), speed - driveBalancer->GetDifference());
}

void DrivetrainSub::resetAHRS()
{
  ahrs->Reset();
}

double DrivetrainSub::getAngle()
{
  return ahrs->GetAngle();
}

double DrivetrainSub::getRate()
{
  return ahrs->GetRate();
}

// Positive means that the robot is tipping forwards.  Negative means that robot is tipping backwards
double DrivetrainSub::getPitchAngle() {
  return ahrs->GetPitch();
}
double DrivetrainSub::getVelocity() {
  return (leftMotor1->GetEncoder().GetVelocity()-rightMotor1->GetEncoder().GetVelocity())/2;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
