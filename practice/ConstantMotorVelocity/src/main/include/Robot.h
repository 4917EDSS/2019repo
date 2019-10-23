/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

// TODO: List all motor CAN IDs here (you have to set the CAN IDs on the motor controllers to match)
constexpr int LEFT_DRIVE_MOTOR_1_CAN_ID = 1;
constexpr int RIGHT_DRIVE_MOTOR_1_CAN_ID = 2;

// TODO: Adjust these test parameters as needed
constexpr double TEST_STARTING_MOTOR_POWER = 1.0;   // Initial motor power before control loop kicks in (-1.0 to 1.0)
constexpr double TEST_TARGET_VELOCITY = 1000;       // Target speed in whatever units the motor encoder reports (this is a wild guess, need to experiment)
constexpr double POWER_ADJUSTMENT_STEP = 0.05;      // Amount of power we add/subtract each iteration when we're going too slow/fast (-1.0 to 1.0)


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;

  // ADDED
  double currentPower;                              // Keep track of current power so we can adjust it up or down
  void setPowerOnAllMotors(double power);           // Easier to add/remove motors with this function

  // TODO:  Create all motor pointers here
  std::shared_ptr <rev::CANSparkMax> leftMotor1;
  std::shared_ptr <rev::CANSparkMax> rightMotor1;
};
