/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <RobotMap.h>
#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>

class ElevatorSub : public frc::Subsystem {
 private:
   std::shared_ptr<frc::Encoder> elevatorMotorEnc;
   std::shared_ptr <rev::CANSparkMax> elevatorMotor;
   double target;



 public:
  ElevatorSub();
  void SetElevatorMotor(double speed);
  void InitDefaultCommand() override;
  void update();
  double getElevatorEncoder();
  void setElevatorMotor(double speed);
  void setTarget(double newTarget);
  bool isFinishedMove();
  bool isElevatorDown();
};
