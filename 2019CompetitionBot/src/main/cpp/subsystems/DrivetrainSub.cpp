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

constexpr float DRIVE_BALANCE_TOLERANCE = 0.5;
constexpr float DRIVE_BALANCE_P = 0;
constexpr float DRIVE_BALANCE_I = 0;
constexpr float DRIVE_BALANCE_D = 0;

DrivetrainSub::DrivetrainSub() : Subsystem("DrivetrainSub"){

  // Todo - use proper CAN ID defines
  rightMotor1.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor2.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor3.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_3_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

  leftMotor1.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor2.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor3.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_3_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

  ahrs.reset(new AHRS(frc::SPI::kMXP));
  ahrs->SetName("Drivetrain", "AHRS");
}

double DrivetrainSub::GetRightEncoder()
{
  return -(rightMotor1->GetEncoder().GetPosition());
}

double DrivetrainSub::GetLeftEncoder()
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

  leftMotor1->Set(lSpeed);
  leftMotor2->Set(lSpeed);
  leftMotor3->Set(lSpeed);

  rightMotor1->Set(-rSpeed);
  rightMotor2->Set(-rSpeed);
  rightMotor3->Set(-rSpeed);
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
	distanceBalancer->Reset();
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

// Put methods for controlling this subsystem
// here. Call these from Commands.
