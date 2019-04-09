/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/Commands/Subsystem.h>
#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <ctre/Phoenix.h>
#include "AHRS.h"
#include "components/MotorBalancer.h"
#include "SparkShuffleboardEntrySet.h"

class DrivetrainSub : public frc::Subsystem {
 private:
  std::shared_ptr <rev::CANSparkMax> leftMotor1;
  std::shared_ptr <rev::CANSparkMax> leftMotor2;
  std::shared_ptr <rev::CANSparkMax> leftMotor3;
  std::shared_ptr <rev::CANSparkMax> rightMotor1;
  std::shared_ptr <rev::CANSparkMax> rightMotor2;
  std::shared_ptr <rev::CANSparkMax> rightMotor3;
  std::shared_ptr<AHRS> ahrs;
  std::shared_ptr<frc::PIDController> driveBalancePID;
  std::shared_ptr<frc4917::MotorBalancer> driveBalancer;

  struct SparkShuffleboardEntrySet nteSparks[6];
  nt::NetworkTableEntry ntePitch;
  nt::NetworkTableEntry nteYaw;
  nt::NetworkTableEntry nteRoll;
  
 public:
  DrivetrainSub();
  double getLeftEncoder();
  double getRightEncoder();
  void resetAHRS();
  double getAngle();
  double getRate();
  double getPitchAngle();
  void InitDefaultCommand() override;
  void drive(double lSpeed, double rSpeed);
  void updateShuffleBoard();
  void driverDriveStraight(float speed);
  void enableBalancerPID();
  void disableBalancerPID();
  double getVelocity();
  void SetDrivetrainEncoderZero();
};
