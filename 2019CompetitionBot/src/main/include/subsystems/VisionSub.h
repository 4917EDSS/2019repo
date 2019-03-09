/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include <frc/commands/Subsystem.h>

class VisionSub : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities

 public:
  VisionSub();
  void InitDefaultCommand() override;
  double getVisionTarget();
  double normalizeAngle(double targetangle);
  void setBumperPipeline(int pipeLine);
  void setManipulatorPipeline(int pipeLine);
  double getDistanceFromVision();
  double getScoringFaceAngle();
};
