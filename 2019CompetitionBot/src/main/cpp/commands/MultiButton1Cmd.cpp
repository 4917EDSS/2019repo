/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/MultiButton1Cmd.h"
#include "Robot.h"
#include "components/Log.h"

// This command gets attached to a single button but execute different functionality based on what
// SHIFT button is pressed.

MultiButton1Cmd::MultiButton1Cmd() {
  // Use Requires() here to declare subsystem dependencies
  // eg. Requires(Robot::chassis.get());

  // Because these for for debug only and are all instantaneous, don't use Requires.
}

// Called just before this Command runs the first time
void MultiButton1Cmd::Initialize() {
  int shift = Robot::oi.getOperatorShiftState();

  switch(shift) {
    case 0:
      if(Robot::manipulatorSub.isGripperExpanded()) {
        Robot::manipulatorSub.contractHatchGripper();
        logger.send(logger.CMD_TRACE, "MultiButton1Cmd: Gripper is contracted.\n");
      }
      else {
        Robot::manipulatorSub.expandHatchGripper();
        logger.send(logger.CMD_TRACE, "MultiButton1Cmd: Gripper is expanded.\n");
      }
      break;

    case 1:
      // Unfold/fold ball intake
      if(Robot::ballIntakeSub.isIntakeUnfolded()) {
        Robot::ballIntakeSub.foldIntakeArms();
        logger.send(logger.CMD_TRACE, "MultiButton1Cmd: Fold ball intake in.\n");
      }
      else {
        Robot::ballIntakeSub.unfoldIntakeArms();
        logger.send(logger.CMD_TRACE, "MultiButton1Cmd: Unfold ball intake.\n");
      }
      break;
    
    case 2:
      // Enable/disable ball intake wheels
      if(Robot::ballIntakeSub.getIntakeWheelPower() == 0) {
        Robot::ballIntakeSub.setIntakeWheelPower(1.0);
        logger.send(logger.CMD_TRACE, "MultiButton1Cmd: Enable ball intake wheels.\n");
      }
      else {
        Robot::ballIntakeSub.setIntakeWheelPower(0.0);
        logger.send(logger.CMD_TRACE, "MultiButton1Cmd: Disable ball intake wheels.\n");
      }
      break;

    default:
      logger.send(logger.CMD_TRACE, "MultiButton1Cmd: Invalid shift value (%d).\n", shift);
      break;
  }
}

// Called repeatedly when this Command is scheduled to run
void MultiButton1Cmd::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool MultiButton1Cmd::IsFinished() { return true; }

// Called once after isFinished returns true
void MultiButton1Cmd::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void MultiButton1Cmd::Interrupted() {
  End();
}
