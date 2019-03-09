/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/ClimbCmd.h"
#include "Robot.h"

constexpr double MAX_ARM_POWER = 0.40;
constexpr double MIN_ARM_POWER = 0.10;
constexpr double ARM_POWER_STEP_SIZE = 0.01;
constexpr double ARM_POWER_ANGLE_TOLERANCE = 2.0;

ClimbCmd::ClimbCmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());
  Requires(&Robot::climbSub);
  Requires(&Robot::ballIntakeSub);

  lastPower = 0.10;
}

// Called just before this Command runs the first time
void ClimbCmd::Initialize() {
  // To climb, unfold the intake arms, start extending the slower climb bars to raise the front of
  // the robot (robot backs in to climb).  Then use the intake arms to raise the rear.  The intake
  // arms are much faster than the climb bars so use the NavX to slow down the intake arms as needed.
  Robot::ballIntakeSub.unfoldIntakeArms();
  
  lastPower = 0.6;
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_MANUAL, lastPower, 0.0);
}

// Called repeatedly when this Command is scheduled to run
void ClimbCmd::Execute() {
  double pitchAngle = Robot::drivetrainSub.getPitchAngle();

  // TODO: Might need a smarter control algorithm
  if(pitchAngle > ARM_POWER_ANGLE_TOLERANCE) {
    // Intake arms aren't keeping up, robot is tipping forward, more power
    lastPower += ARM_POWER_STEP_SIZE;
  }
  else if(pitchAngle < -ARM_POWER_ANGLE_TOLERANCE) {
    // Intake arms are too fast, robot is tipping backward, less power
    lastPower -= ARM_POWER_STEP_SIZE;
  }
  // Otherise, use lastPower without changing it

  // Check power limits before setting new power
  if(lastPower > MAX_ARM_POWER) {
    lastPower = MAX_ARM_POWER;
  }
  else if(lastPower < MIN_ARM_POWER) {
    lastPower = MIN_ARM_POWER;
  }
  
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_MANUAL, lastPower, 0.0);
}


// Make this return true when this Command no longer needs to run execute()
bool ClimbCmd::IsFinished() { 
  if(Robot::climbSub.getClimbPosition() >= CLIMB_EXTEND_LIMIT_THRESHOLD) {
    return true;
  }
  else {
    return false; 
  }
}

// Called once after isFinished returns true
void ClimbCmd::End() {
  Robot::climbSub.SetClimbMotorPower(0.0);
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_MANUAL, 0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ClimbCmd::Interrupted() {
  End();
}
