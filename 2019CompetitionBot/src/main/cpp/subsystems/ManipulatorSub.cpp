/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/WPILib.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <RobotMap.h>
#include "subsystems/ManipulatorSub.h"
#include "components/Log.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInLayouts.h>
#include "SparkShuffleboardEntrySet.h"

constexpr float MANIPULATOR_ANGLE_TOLERANCE = 1.0;
constexpr float FLIPPER_TICK_TO_DEGREE_FACTOR = (90/44.1900);

ManipulatorSub::ManipulatorSub() : Subsystem("ManipulatorSub") {
  flipperMotor.reset(new rev::CANSparkMax(MANIPULATOR_FLIPPER_MOTOR_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  flipperMotor->GetEncoder().SetPosition(0);
  flipperMotor->GetEncoder().SetPositionConversionFactor(FLIPPER_TICK_TO_DEGREE_FACTOR);
  flipperLimit.reset(new frc::DigitalInput(MANIPULATOR_LIMIT_DIO));

  intakeMotorLeft.reset(new WPI_VictorSPX(MANIPULATOR_LEFT_INTAKE_MOTOR_CAN_ID));
  intakeMotorRight.reset(new WPI_VictorSPX(MANIPULATOR_RIGHT_INTAKE_MOTOR_CAN_ID));
  intakeFromRobotLimit.reset(new frc::DigitalInput(BALL_SENSOR_DIO));

  hatchGripperSolenoid.reset(new frc::Solenoid(HATCH_GRIPPER_PCM_ID));
  expandHatchGripper();

  // Setup Shuffleboard for each input and output device
  frc::ShuffleboardTab& shuffleTab = frc::Shuffleboard::GetTab("Manipulator");

  frc::ShuffleboardLayout& shuffleList = shuffleTab.GetLayout("Flipper Motor", frc::BuiltInLayouts::kList);
  nteFlipperMotor.setPower = (shuffleList.Add("Set Power", 0).GetEntry());
  nteFlipperMotor.outputCurrent = (shuffleList.Add("Current Out", 0).GetEntry());
  nteFlipperMotor.encoderPosition = (shuffleList.Add("Position", 0).GetEntry());
  nteFlipperMotor.encoderVelocity = (shuffleList.Add("Velocity", 0).GetEntry());
  nteFlipperMotor.motorTemperature = (shuffleList.Add("Motor Temp", 0).GetEntry());

  nteFlipperLimit = (shuffleTab.Add("Flipper Limit", 0).GetEntry());
  nteIntakeMotorLeft = (shuffleTab.Add("Intake Motor L", 0).GetEntry());
  nteIntakeMotorRight = (shuffleTab.Add("Intake Motor R", 0).GetEntry());
  nteIntakeFromRobotLimit = (shuffleTab.Add("Intake Limit", 0).GetEntry());
  nteHatchGripperSolenoid = (shuffleTab.Add("Gripper", 0).GetEntry());
}

void ManipulatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

void ManipulatorSub::updateShuffleBoard() {
  nteFlipperMotor.setPower.SetDouble(flipperMotor->Get());
  nteFlipperMotor.outputCurrent.SetDouble(flipperMotor->GetOutputCurrent());
  nteFlipperMotor.encoderPosition.SetDouble(flipperMotor->GetEncoder().GetPosition());
  nteFlipperMotor.encoderVelocity.SetDouble(flipperMotor->GetEncoder().GetVelocity());
  nteFlipperMotor.motorTemperature.SetDouble(flipperMotor->GetMotorTemperature());

  nteFlipperLimit.SetBoolean(flipperLimit->Get());
  nteIntakeMotorLeft.SetDouble(intakeMotorLeft->Get());
  nteIntakeMotorRight.SetDouble(intakeMotorRight->Get());
  nteIntakeFromRobotLimit.SetBoolean(intakeFromRobotLimit->Get());
  nteHatchGripperSolenoid.SetBoolean(hatchGripperSolenoid->Get());
}

void ManipulatorSub::setFlipperPower(double power) {
  flipperMotor->Set(power);
}

double ManipulatorSub::getFlipperAngle() {
  return flipperMotor->GetEncoder().GetPosition();
}

double ManipulatorSub::getFlipperVelocity() {
  return flipperMotor->GetEncoder().GetVelocity();
}

bool ManipulatorSub::isFlipperAtLimit() {
  return flipperLimit->Get();
}

void ManipulatorSub::setIntakePower(double power) {
  intakeMotorLeft->Set(-power);
  intakeMotorLeft->Set(power);
}

bool ManipulatorSub::isBallIn() {
  return !intakeFromRobotLimit->Get();
}

void ManipulatorSub::expandHatchGripper(){
  hatchGripperSolenoid->Set(false);
}

void ManipulatorSub::contractHatchGripper(){
  hatchGripperSolenoid->Set(true);
}

bool ManipulatorSub::isGripperExpanded() {
  return !hatchGripperSolenoid->Get();
}








void ManipulatorSub::setManipulatorFlipperMotorSpeed(double speed) {
  if (getManipulatorAngle() < -90 && speed < 0){
    speed = 0;
  }

  else if (getManipulatorAngle() > 90 && speed > 0){
    speed = 0;
  }

   else if (getManipulatorAngle() > 80 && speed > 0){
    speed = std::min(speed, 0.2);
  }
}

void ManipulatorSub::setManipulatorTargetAngle(double newAngle) {
  targetAngle = newAngle;
}

void ManipulatorSub::holdManipulatorFlipper(double position){
  double holdvalue = (position -  flipperMotor->GetEncoder().GetPosition());
  setFlipperPower(holdvalue*0.1); 
}

double ManipulatorSub::getManipulatorAngle(){
  return  targetAngle;
}

bool ManipulatorSub::isFinishedMove() {
if (fabs(targetAngle -flipperMotor->GetEncoder().GetPosition()) < MANIPULATOR_ANGLE_TOLERANCE && fabs(flipperMotor->GetEncoder().GetVelocity()) < 45) {
      return true;
   } else {
     return false;
   }
}

void ManipulatorSub::zeroEverything(){
  intakeMotorLeft->Set(0.0);
  intakeMotorRight->Set(0.0);
  flipperMotor->Set(0.0);
}
