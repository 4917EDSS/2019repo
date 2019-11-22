/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // TODO:  Create all motors here
  leftMotor1.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor2.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  leftMotor3.reset(new rev::CANSparkMax(LEFT_DRIVE_MOTOR_3_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor1.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor2.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  rightMotor3.reset(new rev::CANSparkMax(RIGHT_DRIVE_MOTOR_3_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  currentPower = 0;
}

void Robot::setPowerOnAllMotors(double power) {
  if(power > 1.0) {
    power = 1.0;
  }
  else if(power < -1.0) {
    power = -1.0;
  }

  // TODO: Set same power for all motors and make sure direction is correct for each motor
  leftMotor1->Set(power);
  leftMotor2->Set(power);
  leftMotor3->Set(power);
  rightMotor1->Set(-power);
  rightMotor2->Set(-power);
  rightMotor3->Set(-power); // Assume that motor needs to turn in opposite direction from left side to go forward

  currentPower = power;
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

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
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  // ADDED: This is where we set all the motors' power.  This probably doesn't need to change.
  setPowerOnAllMotors(TEST_STARTING_MOTOR_POWER);
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
  std::cout << leftMotor1->GetEncoder().GetVelocity() << "\n";
  // TODO: Modify this control loop as needed
  //   Notes - This is a super cheesy way to implement a control loop.  A PID would be better
  //           but that requires a lot of tuning for each experiment setup.
  //         - Don't set the ticks/sec to distance/time conversion on motor controller, it can 
  //           loose this conversion factor during brown-outs.  Use raw encoder ticks and 
  //           convert right here if necessary
  //           (i.e. if you want something like mm/s instead of raw RPM units)
  if( leftMotor1->GetEncoder().GetVelocity() > TEST_TARGET_VELOCITY ) {
    // Going too fast, slow down
    setPowerOnAllMotors(currentPower - POWER_ADJUSTMENT_STEP);
  }
  else {
    // Going too slow, speed up
    setPowerOnAllMotors(currentPower + POWER_ADJUSTMENT_STEP);
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
