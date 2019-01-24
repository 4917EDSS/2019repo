/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MilkyManipulatorWithJoystickCmd.h"
#include "OI.H"
#include <iostream>

MilkyManipulatorWithJoystickCmd::MilkyManipulatorWithJoystickCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  //Requires(MilkyManipulatorWithJoystickCmd.get());
}

// Called just before this Command runs the first time
void MilkyManipulatorWithJoystickCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void MilkyManipulatorWithJoystickCmd::Execute() {
  //std::share_ptr<frc::Joystick> operatorJoystick = oi->getOperatorController();
  //MilkyManipulatorWithJoystickSub->isBallIn();

  //double scaledVerticalStick = pow (verticalStick, 3);
  
 // MilkyManipulatorWithJoystickCmd->milkyManipulator(scaledVerticalStick);
}

// Make this return true when this Command no longer needs to run execute()
bool MilkyManipulatorWithJoystickCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void MilkyManipulatorWithJoystickCmd::End() {
  //MilkyManipulatorWithJoystickCmd->milkyManipulator(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MilkyManipulatorWithJoystickCmd::Interrupted() {
  End();
}
