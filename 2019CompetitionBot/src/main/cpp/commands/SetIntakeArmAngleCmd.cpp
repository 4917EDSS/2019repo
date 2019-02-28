/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetIntakeArmAngleCmd.h"
#include "Robot.h"
#include <iostream>

SetIntakeArmAngleCmd::SetIntakeArmAngleCmd(bool isClimbing, double angle)  {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::ballIntakeSub);

  isClimbingNow = isClimbing;
  targetAngle = angle;
}

SetIntakeArmAngleCmd::SetIntakeArmAngleCmd(double angle) {
  SetIntakeArmAngleCmd(false, angle);
}

// Called just before this Command runs the first time
void SetIntakeArmAngleCmd::Initialize() {
  logger.send(logger.CMD_TRACE, "%s : %s | Angle = %.1f\n", __FILE__, __FUNCTION__, targetAngle);
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_AUTO, targetAngle, 1.0);
}

// Called repeatedly when this Command is scheduled to run
void SetIntakeArmAngleCmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool SetIntakeArmAngleCmd::IsFinished() {  
    return Robot::ballIntakeSub.isIntakeArmAtTarget(); 
}

// Called once after isFinished returns true
void SetIntakeArmAngleCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetIntakeArmAngleCmd::Interrupted() {
  End();
}
