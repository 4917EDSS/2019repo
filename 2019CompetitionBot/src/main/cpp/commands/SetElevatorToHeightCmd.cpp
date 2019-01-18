/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/SetElevatorToHeightCmd.h"
#include "Robot.h"

SetElevatorToHeightCmd::SetElevatorToHeightCmd(double height) : height(height)
{
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::elevatorSub);
}

// Called just before this Command runs the first time
void SetElevatorToHeightCmd::Initialize()
{
  Robot::elevatorSub.setTarget(height);
}

// Called repeatedly when this Command is scheduled to run
void SetElevatorToHeightCmd::Execute()
{
  if (height == 0)
  {
    Robot::elevatorSub.setElevatorMotor(-1.0);
  }
  else
  {
    Robot::elevatorSub.update();
  }
}

// Make this return true when this Command no longer needs to run execute()
bool SetElevatorToHeightCmd::IsFinished()
{
  if (TimeSinceInitialized() > 5.0)
  {
    return true;
  }
  return Robot::elevatorSub.isFinishedMove();
}

// Called once after isFinished returns true
void SetElevatorToHeightCmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SetElevatorToHeightCmd::Interrupted() {}