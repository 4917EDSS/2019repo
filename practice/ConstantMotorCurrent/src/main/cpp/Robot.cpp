/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

Robot::Robot() {
  // Note SmartDashboard is not initialized here, wait until RobotInit() to make
  // SmartDashboard calls
  m_robotDrive.SetExpiration(0.1);

  leftMotor1.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor1.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));

  leftMotor1->SetSmartCurrentLimit(MAX_CURRENT);
  rightMotor1->SetSmartCurrentLimit(MAX_CURRENT);

}

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);


}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::Autonomous() {
  std::cout << "Auto mode running.  Setting motor(s) to fixed value." << std::endl;

  // MotorSafety improves safety when motors are updated in loops but is
  // disabled here because motor updates are not looped in this autonomous mode.
  m_robotDrive.SetSafetyEnabled(false);

  leftMotor1->Set(1.0);
  rightMotor1->Set(1.0);
}

/**
 * Runs the motors with arcade steering.
 */
void Robot::OperatorControl() {
  m_robotDrive.SetSafetyEnabled(true);
  while (IsOperatorControl() && IsEnabled()) {
    // Drive with arcade style (use right stick)
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());

    // The motors will be updated every 5ms
    frc::Wait(0.005);
  }
}

/**
 * Runs during test mode
 */
void Robot::Test() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
