/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"

#include <frc/WPILib.h>
#include "Robot.h"
#include "subsystems/BallIntakeSub.h"
#include "commands/KillEverythingCmd.h"
#include "commands/MilkyScoreGrp.h"
#include "commands/TestButtonCmd.h"
#include "commands/ClimbExtendGrp.h"
#include "commands/IntakeBallFromRobotCmd.h"
#include "commands/IntakeBallGrp.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/MultiButton1Cmd.h"
#include "commands/HatchModeGrp.h"
#include "commands/CargoModeGrp.h"
#include "commands/ToggleManipulatorPositionCmd.h"
#include "commands/SetElevatorToHeightCmd.h"
#include "commands/ExtendClimbBarsCmd.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/RetractClimbBarsCmd.h"
#include "commands/ModeBasedCndCmd.h"
#include "commands/ClimbCmd.h"
#include "commands/SideBaseCmd.h"
#include "commands/FlipManipulatorGrp.h"
#include "commands/ZeroAllSystemsGrp.h"
#include "commands/ClimbBarAutoRetractCmd.h"
#include "commands/ClimbRetractGrp.h"
#include "commands/CargoHighHeightGrp.h"
#include "commands/AutoLeftRocketCloseHatchPart2Grp.h"
#include "commands/AutoRightRocketCloseHatchPart2Grp.h"
#include "commands/VisionHatchPickupGrp.h"
#include "commands/VisionScoringCmd.h"
#include "commands/ClimbAgainGrp.h"
#include "commands/LowVisionScoreGrp.h"



OI::OI() {
  // Process operator interface input here.
  driverController.reset(new frc::Joystick(DRIVER_CONTROLLER_PORT));
  driverController->SetXChannel(0);
  driverController->SetYChannel(1);
  driverController->SetZChannel(2);
  driverController->SetThrottleChannel(3);

  operatorController.reset(new frc::Joystick(OPERATOR_CONTROLLER_PORT));
  operatorController->SetXChannel(0);
  operatorController->SetYChannel(1);
  operatorController->SetZChannel(2);
  operatorController->SetThrottleChannel(3);

  // Driver controller buttons
  driveToVisionTargetBtn.reset(new frc::JoystickButton(driverController.get(), DRIVE_TO_VISION_TARGET_BTN));
  driveToVisionTargetBtn->WhenPressed(new VisionScoringCmd());

  driveToVisionTargetWithManipulatorBtn.reset(new frc::JoystickButton(driverController.get(), DRIVE_TO_VISION_TARGET_WITH_MANIPULATOR_BTN));
  driveToVisionTargetWithManipulatorBtn->WhenPressed(new VisionHatchPickupGrp());

  scoreLowVision.reset(new frc::JoystickButton(driverController.get(), SCORE_LOW_WITH_VISION_BTN));
  scoreLowVision->WhenPressed(new LowVisionScoreGrp());

  climbBtn.reset(new frc::JoystickButton(driverController.get(), CLIMB_BTN));
  climbBtn->WhenPressed(new ClimbExtendGrp());

  extendClimbBarsBtn.reset(new frc::JoystickButton(driverController.get(), EXTEND_CLIMB_BARS_BTN)); 
  extendClimbBarsBtn->WhileHeld(new ExtendClimbBarsCmd());

  reverseClimbBtn.reset(new frc::JoystickButton(driverController.get(), REVERSE_CLIMB_BTN));
  reverseClimbBtn->WhenPressed(new ClimbRetractGrp());

  retractClimbBarsBtn.reset(new frc::JoystickButton(driverController.get(), RETRACT_CLIMB_BARS_BTN));
  retractClimbBarsBtn->WhileHeld(new RetractClimbBarsCmd());

  AutoLeftRocketCloseHatchPart2GrpBtn.reset(new frc::JoystickButton(driverController.get(), CONTINUE_LEFT_AUTO_BTN));
  AutoLeftRocketCloseHatchPart2GrpBtn->WhenPressed(new AutoLeftRocketCloseHatchPart2Grp());
  
  AutoRightRocketCloseHatchPart2GrpBtn.reset(new frc::JoystickButton(driverController.get(), CONTINUE_RIGHT_AUTO_BTN));
  AutoRightRocketCloseHatchPart2GrpBtn->WhenPressed(new AutoRightRocketCloseHatchPart2Grp());

  driverKillBtn1.reset(new frc::JoystickButton(driverController.get(), DRIVER_KILL_ONE_BTN));
  driverKillBtn1->WhenPressed(new KillEverythingCmd());

  driverKillBtn2.reset(new frc::JoystickButton(driverController.get(), DRIVER_KILL_TWO_BTN));
  driverKillBtn2->WhenPressed(new KillEverythingCmd());

  climbAgainBtn.reset(new frc::JoystickButton(driverController.get(), CLIMB_AGAIN_BTN));
  climbAgainBtn->WhenPressed(new ClimbAgainGrp());
  // Operator controller buttons
  elevatorToCargoShipHeightBtn.reset(new frc::JoystickButton(operatorController.get(), ELEVATOR_TO_CARGO_SHIP_HEIGHT_BTN));
  elevatorToCargoShipHeightBtn->WhenPressed( new SideBaseCmd (new SetElevatorToHeightCmd(ELEVATOR_CARGO_SHIP_CARGO_HEIGHT_MM), new SetElevatorToHeightCmd(ELEVATOR_MAX_SAFE_HEIGHT_MANIPULATOR_TO_REAR-1.0)));

  elevatorToLowHeightBtn.reset(new frc::JoystickButton(operatorController.get(), ELEVATOR_TO_LOW_HEIGHT_BTN));
  elevatorToLowHeightBtn->WhenPressed(new ModeBasedCndCmd(new SideBaseCmd(new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM), new SetElevatorToHeightCmd(ELEVATOR_LOW_HATCH_HEIGHT_MM + 100.0)), new SetElevatorToHeightCmd(ELEVATOR_ROCKET_LOW_CARGO_HEIGHT_MM)));
  
  elevatorToMediumHeightBtn.reset(new frc::JoystickButton(operatorController.get(), ELEVATOR_TO_MEDIUM_HEIGHT_BTN));
  elevatorToMediumHeightBtn->WhenPressed(new ModeBasedCndCmd(new SetElevatorToHeightCmd(ELEVATOR_MEDIUM_HATCH_HEIGHT_MM), new SetElevatorToHeightCmd(ELEVATOR_ROCKET_MEDIUM_CARGO_HEIGHT_MM)));

  elevatorToHighHeightBtn.reset(new frc::JoystickButton(operatorController.get(), ELEVATOR_TO_HIGH_HEIGHT_BTN));
  elevatorToHighHeightBtn->WhenPressed(new ModeBasedCndCmd(new SetElevatorToHeightCmd(ELEVATOR_HIGH_HATCH_HEIGHT_MM), new CargoHighHeightGrp()));

  hatchModeBtn.reset(new frc::JoystickButton(operatorController.get(), HATCH_MODE_BTN));
  hatchModeBtn->WhenPressed(new HatchModeGrp());

  cargoModeBtn.reset(new frc::JoystickButton(operatorController.get(), CARGO_MODE_BTN));
  cargoModeBtn->WhenPressed(new CargoModeGrp());

  flipManipulatorBtn.reset(new frc::JoystickButton(operatorController.get(), FLIP_MANIPULATOR_BTN));
  flipManipulatorBtn->WhenPressed(new FlipManipulatorGrp());

  intakeHatchOrCargoBtn.reset(new frc::JoystickButton(operatorController.get(), INTAKE_HATCH_OR_CARGO_BTN));
  intakeHatchOrCargoBtn->WhenPressed(new ModeBasedCndCmd(new ExpandHatchGripperGrp(), new IntakeBallGrp()));

  zeroAllSystemsBtn.reset(new frc::JoystickButton(operatorController.get(), ZERO_ALL_SYSTEMS_BTN));
  zeroAllSystemsBtn->WhenPressed(new ZeroAllSystemsGrp());

  multiCommand1Btn.reset(new frc::JoystickButton(operatorController.get(), MULTI_COMMAND_1_BTN));
  multiCommand1Btn->WhenPressed(new MultiButton1Cmd());

  operatorKillBtn1.reset(new frc::JoystickButton(operatorController.get(), OPERATOR_KILL_ONE_BTN));
  operatorKillBtn1->WhenPressed(new KillEverythingCmd());

  operatorKillBtn2.reset(new frc::JoystickButton(operatorController.get(), OPERATOR_KILL_TWO_BTN));
  operatorKillBtn2->WhenPressed(new KillEverythingCmd());



/*
  // These commands don't yet have a button

  climbModeBtn.reset(new frc::JoystickButton(operatorController.get(), CLIMB_MODE_BTN));
  climbModeBtn->WhileHeld(new ClimbCmdGroup());


  // These commands are likley obsolete

  intakeBallBtn.reset(new frc::JoystickButton(operatorController.get(), INTAKE_BALL_BTN));
  intakeBallBtn->WhenPressed(new IntakeBallGrp());
  
  IntakeUntilLimitBtn.reset(new frc::JoystickButton(operatorController.get(), SET_INTAKE_MOTOR_BTN));
  IntakeUntilLimitBtn->WhenPressed(new IntakeBallFromRobotCmd());
*/

}

std::shared_ptr<frc::Joystick> OI::getDriverController() {
	return driverController;
}

std::shared_ptr<frc::Joystick> OI::getOperatorController() {
  return operatorController;
}

// Use the POV (i.e. D-Pad) as a command modifier
int OI::getOperatorShiftState() {
  int shift = 0;

  // POV position is reported in degrees with 0 deg being up and increasing clockwise
  // Positions between Up/down/left/right are ignored (i.e. 45 deg)
  switch(operatorController->GetPOV()) {
    case POV_INACTIVE:
      shift = 0;
      break;
    case POV_UP:
      shift = 1;
      break; 
    case POV_RIGHT:
      shift = 2;
      break;
    case POV_DOWN:
      shift = 3;
      break;
    case POV_LEFT:
      shift = 4;
      break;
  }

  return shift;
}
