/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/AreaFiftyOneSub.h"
#include "commands/MarioKartMobileCmd.h"
#include "Robot.h"

MarioKartMobileCmd::MarioKartMobileCmd() {
  Requires (&Robot::areaFiftyOneSub);
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void MarioKartMobileCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MarioKartMobileCmd::Execute() {
  std::shared_ptr<frc::Joystick> driverJoystick =
    Robot::m_oi.getDriverController();
  Robot::areaFiftyOneSub.drive(driverJoystick->GetY(),
    driverJoystick->GetZ());
}

// Make this return true when this Command no longer needs to run execute()
bool MarioKartMobileCmd::IsFinished() {
   return false; }

// Called once after isFinished returns true
void MarioKartMobileCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MarioKartMobileCmd::Interrupted() {}
