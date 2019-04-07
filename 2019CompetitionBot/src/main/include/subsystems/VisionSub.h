/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/WPILib.h>
#include <frc/commands/Subsystem.h>
#include <utility>

constexpr int DRIVER_MODE_NORMAL = 1;
constexpr int DRIVER_MODE_FLIPPED = 2;
constexpr int VISION_MODE_NORMAL = 3;
constexpr int VISION_MODE_FLIPPED = 4;

constexpr int BUMPER_CAMERA = 1;
constexpr int MANIPULATOR_CAMERA = 2;

class VisionSub : public frc::Subsystem {
 private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
  int setManipulatorPipelineState;
 public:
  VisionSub();
  void InitDefaultCommand() override;
  int getManipulatorPipeline();
  double getVisionTarget(int camera);
  double getRobotTargetAngle(double robotHeading, double cameraAngle, double scoringFaceAngle);
  std::pair<double, double> normalizeAngle(double angle);
  void setBumperPipeline(int pipeLine);
  void setManipulatorPipeline(int pipeLine);
  bool isTargetVisible(int camera);
  double getScoringFaceAngle(int camera);
  double getVerticalOffset(int camera);
  std::shared_ptr<NetworkTable> getTable(int camera);
};
