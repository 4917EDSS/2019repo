/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/BallIntakeSub.h"
#include <ctre/Phoenix.h>
#include "components/Log.h"
#include <iostream>
#include "commands/BallIntakeWithJoystickCmd.h"
#include "SparkShuffleboardEntrySet.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "frc/shuffleboard/BuiltInLayouts.h"
//Canid4 and Set Intake
#define ENCODER_SCALE (90.0 / 32.0)

constexpr double INTAKE_ARM_MAX_ANGLE = 150; //change
constexpr double INTAKE_ARM_MIN_ANGLE = 0;
constexpr double INTAKE_ARM_ANGLE_TOLERANCE = 1.0;
constexpr double INTAKE_ARM_VELOCITY_TOLERANCE = 45;
constexpr double INTAKE_ARM_TICK_TO_DEGREE_FACTOR = (90.0 / 32.0);
constexpr double MANUAL_MODE_POWER_DEADBAND = 0.03;

// Manipulator state machine states
constexpr int INTAKE_ARM_STATE_IDLE = 0;
constexpr int INTAKE_ARM_STATE_HOLDING = 1;
constexpr int INTAKE_ARM_STATE_MOVING = 2;
constexpr int INTAKE_ARM_STATE_INTERRUPTED = 3;

BallIntakeSub::BallIntakeSub() : Subsystem("BallIntakeSub")
{
  ballIntakeMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(BALL_INTAKE_WHEELS_MOTOR_CAN_ID));
  currentSpeed = 0;
  flipperMotorOne.reset(new rev::CANSparkMax(BALL_INTAKE_TOP_FLIP_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  intakeFolderSolenoid.reset(new frc::Solenoid(BALL_INTAKE_FOLDER_PCM_ID));
  ballIntakeArmLimit.reset(new frc::DigitalInput(BALL_INTAKE_ARM_LIMIT_DIO));
  setFolderOut(true);
  intakeArmEnc.reset(new frc::Encoder(INTAKE_MOTOR_ENC1_DIO, INTAKE_MOTOR_ENC2_DIO));

  frc::ShuffleboardTab &shuffleTab = frc::Shuffleboard::GetTab("Ball Intake");

  nteBallIntakeMotor = (shuffleTab.Add("Intake", 0).GetEntry());
  nteFlipperMotorOne = (shuffleTab.Add("Flipper 1", 0).GetEntry());
  nteFlipperMotorOne = (shuffleTab.Add("Flipper 2", 0).GetEntry());
  nteIntakeFolderSolenoid = (shuffleTab.Add("Folder", 0).GetEntry());
  nteBallIntakeArmLimit = (shuffleTab.Add("Intake Limit", 0).GetEntry());
  nteIntakeArmEncPosition = (shuffleTab.Add("Intake Position", 0).GetEntry());
}

void BallIntakeSub::updateShuffleBoard()
{
  nteBallIntakeMotor.SetDouble(ballIntakeMotor->Get());
  nteFlipperMotorOne.SetDouble(flipperMotorOne->Get());
  nteIntakeFolderSolenoid.SetBoolean(intakeFolderSolenoid->Get());
  nteBallIntakeArmLimit.SetBoolean(ballIntakeArmLimit->Get());
  nteIntakeArmEncPosition.SetDouble(intakeArmEnc->Get());
}

void BallIntakeSub::InitDefaultCommand()
{
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new BallIntakeWithJoystickCmd());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.

void BallIntakeSub::setIntakeWheelsMotorSpeed(double speed)
{
  ballIntakeMotor->Set(ControlMode::PercentOutput, speed);
  currentSpeed = speed;
  logger.send(logger.DEBUGGING, "%s\n", __FUNCTION__);
}

double BallIntakeSub::getIntakeWheelsMotorSpeed()
{
  return currentSpeed;
}

double BallIntakeSub::getIntakeArmEncoderAngle()
{
  //This assumes 5 encoder ticks per degree, this will need to be tested

  return -(intakeArmEnc->GetDistance() * ENCODER_SCALE);
}

bool BallIntakeSub::getIntakeArmLimit()
{
  return !ballIntakeArmLimit->Get();
}

void BallIntakeSub::setFolderOut(bool flipOut)
{
  intakeFolderSolenoid->Set(!flipOut);

  logger.send(logger.DEBUGGING, "%s\n", __FUNCTION__);
}

bool BallIntakeSub::isFolderOut()
{
  return !intakeFolderSolenoid->Get();
}

void BallIntakeSub::setIntakeArmPower(double speed, bool isClimbing)
{
  flipperMotorOne->Set(ControlMode::PercentOutput, -speed);
}

bool BallIntakeSub::getIntakeArmSpeed()
{
  return flipperMotorOne->GetEncoder().GetVelocity();
}

void BallIntakeSub::setArmTargetPosition(int mode, double maxPower, double targetAngle)
{
  //only do something if the intake arm parameters have changed since last call
  if ((mode != intakeArmControlMode) || (maxPower != intakeArmMaxPower) || (intakeAngle != intakeArmTargetAngle))
  {
    // if an old input is pending, drop it
    intakeArmNewStateParameters = false;

    // Check input parameters
    if (fabs(maxPower) > 1.0)
    {
      return;
    }
    if ((targetAngle < INTAKE_ARM_MIN_ANGLE) || (targetAngle > INTAKE_ARM_MAX_ANGLE))
    {
      return;
    }

    switch (mode)
    {
    case INTAKE_ARM_MODE_DISABLED:
      // Turn intake motors off
      intakeArmNewState = INTAKE_ARM_STATE_IDLE;
      intakeArmNewControlMode = mode;
      intakeArmNewMaxPower = 0.0;
      intakeArmNewTargetAngle = 0.0;
      intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
      //logger.send(logger.FLIPPER, "SFA: Disabled\n");
      break;

    case INTAKE_ARM_MODE_AUTO:
      // Go to new target position
      intakeArmNewState = INTAKE_ARM_STATE_MOVING;
      intakeArmNewControlMode = mode;
      intakeArmNewMaxPower = fabs(maxPower);
      intakeArmNewTargetAngle = targetAngle;
      intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
                                          // logger.send(logger.MANIPULATOR, "SFA: Auto (P=%.2f, A=%.1f)\n", flipperNewMaxPower, flipperNewTargetAngle);
      break;

    case INTAKE_ARM_MODE_MANUAL:
      // Take the joystick input to determine the direction of travel and power
      // Note that in this mode the power is continually adjusted (i.e. this function gets called a lot).
      if (fabs(maxPower) < MANUAL_MODE_POWER_DEADBAND)
      {
        // Hold position
        if (intakeArmState != INTAKE_ARM_STATE_HOLDING)
        {
          intakeArmNewState = INTAKE_ARM_STATE_HOLDING;
          intakeArmNewControlMode = mode;
          intakeArmNewMaxPower = 0.0;
          intakeArmNewTargetAngle = getIntakeArmEncoderAngle();
          intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
          //logger.send(logger.MANIPULATOR, "SFA: Man - deadzone start @ %f\n", flipperNewTargetAngle);
        }
      }
      else
      {
        // Move using specified power
        intakeArmNewState = INTAKE_ARM_STATE_MOVING;
        intakeArmNewControlMode = mode;
        intakeArmNewMaxPower = fabs(maxPower);
        intakeArmNewTargetAngle = (maxPower > 0) ? INTAKE_ARM_MAX_ANGLE : INTAKE_ARM_MIN_ANGLE;
        intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
        //logger.send(logger.MANIPULATOR, "SFA: Man - moving (P=%.2f, H=%.1f)\n", flipperNewMaxPower, flipperNewTargetAngle);
      }
      break;
    }
  }
}

void BallIntakeSub::updateIntakeArmStateMachine()
{
  double newPower = 0.0;
  double currentAngle = getIntakeArmEncoderAngle();

  // Apply any new inputs
  if (intakeArmNewStateParameters)
  {
    intakeArmState = intakeArmNewState;
    intakeArmControlMode = intakeArmNewControlMode;
    intakeArmMaxPower = intakeArmNewMaxPower;
    intakeArmTargetAngle = intakeArmNewTargetAngle;
    intakeArmNewStateParameters = false;
    //logger.send(logger.MANIPULATOR, "FSM: New (S=%d, M=%d, P=%.2f, A=%.1f)\n",
    //   flipperState, flipperControlMode, flipperNewMaxPower, flipperNewTargetAngle);
  }

  // In auto mode
  //  - The target angle and max power are set by the caller
  // In manual mode
  // - If the joystick is outside the dead zone, the target angle is set to the highest/lowest
  //   allowable angle with the max power based on the joystick value.
  // - If the max power is close to zero (in the dead zone), the target angle is set to the
  //   current angle.
  switch (intakeArmState)
  {
  case INTAKE_ARM_STATE_IDLE:
    // Don't drive motors, leave as is
    newPower = 0.0;
    break;

  case INTAKE_ARM_STATE_HOLDING:
    // Give the motor just enough power to keep the current position
    newPower = calcIntakeArmHoldPower(currentAngle, intakeArmTargetAngle);

    logger.send(logger.MANIPULATOR, "FSM: Holding (P=%.2f)\n", newPower);
    break;

  case INTAKE_ARM_STATE_MOVING:
    if (isIntakeArmBlocked(currentAngle, intakeArmTargetAngle))
    {
      intakeArmBlockedAngle = currentAngle;
      intakeArmState = INTAKE_ARM_STATE_INTERRUPTED;
    }
    else if (isIntakeArmAtTarget())
    {
      intakeArmState = INTAKE_ARM_STATE_HOLDING;
    }
    else
    {
      newPower = calcIntakeArmMovePower(currentAngle, intakeArmTargetAngle, intakeArmMaxPower);
    }
    logger.send(logger.MANIPULATOR, "FSM: Moving (P=%.2f,s=%d,b=%.1f,c=%.1f)\n", newPower, flipperState, flipperBlockedAngle, currentAngle);
    break;

  case INTAKE_ARM_STATE_INTERRUPTED:
    if (!isIntakeArmBlocked(currentAngle, intakeArmTargetAngle))
    {
      flipperState = INTAKE_ARM_STATE_MOVING;
    }
    else
    {
      newPower = calcIntakeArmHoldPower(currentAngle, flipperBlockedAngle);
    }
    //logger.send(logger.MANIPULATOR, "FSM: Blocked (P=%.2f,s=%d,b=%.1f,c=%.1f)\n", newPower, flipperState, flipperBlockedAngle, currentAngle);
    break;

  default:
    // This should never happen.  Best thing we can do is hold our current position.
    //logger.send(logger.ASSERTS, "Flipper state machine entered invalid state (%d).  Fix the code!\n", flipperState);
    flipperState = INTAKE_ARM_STATE_HOLDING;
    break;
  }

  if (newPower != intakeArmLastPower)
  {
    setIntakeArmPower(newPower, false);
    intakeArmLastPower = newPower;
    //logger.send(logger.MANIPULATOR, "FSM: New power = %f (S=%d, M=%d, P=%.2f, A=%.1f)\n", newPower,
    //flipperState, flipperControlMode, flipperNewMaxPower, flipperNewTargetAngle);
  }
}

bool BallIntakeSub::isIntakeArmBlocked(double currentAngle, double targetAngle)
{
  double direction = 1.0;

  if (currentAngle > targetAngle)
  {
    direction = -1.0;
  }

  // TODO: implement all rules
  if (((currentAngle >= INTAKE_ARM_MAX_ANGLE) && (direction > 0)) ||
      ((currentAngle < INTAKE_ARM_MIN_ANGLE) && (direction < 0)))
  {
    return true;
  }

  return false;
}

double BallIntakeSub::calcIntakeArmHoldPower(double currentAngle, double targetAngle)
{
  return ((targetAngle - currentAngle) * 0.01);
}

bool BallIntakeSub::isIntakeArmAtTarget()
{
  if ((fabs(intakeArmTargetAngle - getIntakeArmEncoderAngle()) < INTAKE_ARM_ANGLE_TOLERANCE) &&
      (fabs(getIntakeWheelsMotorSpeed() < INTAKE_ARM_VELOCITY_TOLERANCE)))
  {
    return true;
  }
  else
  {
    return false;
  }
}

double BallIntakeSub::calcIntakeArmMovePower(double currentAngle, double targetAngle, double maxPower)
{
  double direction = 1.0;
  double newPower = 0;

  if (currentAngle > targetAngle)
  {
    direction = -1.0;
  }

  // TODO: Use better values
  if (fabs(currentAngle - targetAngle) > 10)
  {
    newPower = maxPower * direction;
  }
  else
  {
    newPower = std::min(0.05, maxPower) * direction;
  }

  return newPower;
}

bool BallIntakeSub::doneFlipping()
{
  return false;
}
