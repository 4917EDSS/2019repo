/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once


#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "OI.h"
#include "subsystems/DrivetrainSub.h"
#include "subsystems/BallIntakeSub.h"
#include "subsystems/ElevatorSub.h"
#include "subsystems/ManipulatorSub.h"
#include "subsystems/ClimbSub.h"
#include "subsystems/VisionSub.h"
#include "components/Log.h"

class Robot : public frc::TimedRobot {
 public:
  static DrivetrainSub drivetrainSub;
  static BallIntakeSub ballIntakeSub;
  static ElevatorSub elevatorSub;
  static ManipulatorSub manipulatorSub;
  static ClimbSub climbSub;
  static VisionSub visionSub;
  static OI oi;


  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  static double GetVisionTarget();
  static double NormalizeAngle(double targetangle);
  static void pipeLineToggle(bool pipeLine);
  static void pipeLineFlip(bool pipeLine);

  static bool inBallMode;

 private:
  static bool stateMachinesReset;

  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = nullptr;
  frc::SendableChooser<frc::Command*> m_chooser;
  void UpdateSmartDashboard();
  static void resetStateMachines();
};
