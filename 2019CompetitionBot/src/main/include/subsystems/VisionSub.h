/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include <frc/commands/Subsystem.h>

constexpr int DRIVER_MODE_NORMAL = 1;
constexpr int DRIVER_MODE_FLIPPED = 2;
constexpr int VISION_MODE_NORMAL = 3;
constexpr int VISION_MODE_FLIPPED = 4;

class VisionSub : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  int setManipulatorPipelineState;
 public:
  VisionSub();
  void InitDefaultCommand() override;
  int getManipulatorPipeline();
  double getVisionTarget();
  double normalizeAngle(double targetangle);
  void setBumperPipeline(int pipeLine);
  void setManipulatorPipeline(int pipeLine);
  double getDistanceFromVision();
  double getScoringFaceAngle();
};
