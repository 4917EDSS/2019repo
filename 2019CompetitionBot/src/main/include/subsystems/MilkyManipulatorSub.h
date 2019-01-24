/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>

class MilkyManipulatorSub : public frc::Subsystem {
 private:
  std::shared_ptr(WPI_VictorSPX) milkyManipulatorMotorLeft;
  std::shared_ptr(WPI_VictorSPX) milkyManipulatorMotorRight;
  std::shared_ptr(WPI_VictorSPX) milkyManipulatorFlipperMotor;



  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  MilkyManipulatorSub();
  void InitDefaultCommand() override;
  void milkyManipulator(double speed);
  //sets the speed for the intake on the manipulator
  void milkyManipulator(double leftSpeed, double rightSpeed);
  //sets the two sperate motors on the manipulator
  void setClawPosition(enum clawPosition);
  //finds and sets the claw pickup
  void flip(enum flipPosition);
  //flips the manipulator arms
  void currentClawPosition(bool currentClawPosition);
  // finds out the current claw position
};
