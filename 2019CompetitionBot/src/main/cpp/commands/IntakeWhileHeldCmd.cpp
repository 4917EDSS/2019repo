/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/IntakeWhileHeldCmd.h"
#include "robot.h"

IntakeWhileHeldCmd::IntakeWhileHeldCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void IntakeWhileHeldCmd::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void IntakeWhileHeldCmd::Execute() {
  Robot::ballIntakeSub.SetIntakeMotor(-1.0);
  logger.send(logger.DEBUGGING, "%s : %s\n", __FILE__, __FUNCTION__);
}


// Make this return true when this Command no longer needs to run execute()constexpr int ELEVATOR_MOTOR_CAN_ID = 8;
bool IntakeWhileHeldCmd::IsFinished() { return false; }

// Called once after isFinished returns true
void IntakeWhileHeldCmd::End() {
  Robot::ballIntakeSub.SetIntakeMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void IntakeWhileHeldCmd::Interrupted() {
  IntakeWhileHeldCmd::End();
}
