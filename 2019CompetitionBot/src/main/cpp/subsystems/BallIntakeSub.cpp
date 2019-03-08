/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/BallIntakeSub.h"
#include "components/Log.h"
#include "commands/BallIntakeWithJoystickCmd.h"
#include "SparkShuffleboardEntrySet.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "frc/shuffleboard/BuiltInLayouts.h"
#include <iostream>

constexpr double INTAKE_ARM_MAX_ANGLE = 150; //change
constexpr double INTAKE_ARM_MIN_ANGLE = 0;
constexpr double INTAKE_ARM_ANGLE_TOLERANCE = 1.0;
constexpr double INTAKE_ARM_VELOCITY_TOLERANCE = 45;
constexpr double INTAKE_ARM_TICK_TO_DEGREE_FACTOR = (-90.0 / 39.86);
constexpr double MANUAL_MODE_POWER_DEADBAND = 0.03;

// Intake arm state machine states
constexpr int INTAKE_ARM_STATE_IDLE = 0;
constexpr int INTAKE_ARM_STATE_HOLDING = 1;
constexpr int INTAKE_ARM_STATE_MOVING = 2;
constexpr int INTAKE_ARM_STATE_INTERRUPTED = 3;

BallIntakeSub::BallIntakeSub() : Subsystem("BallIntakeSub") {
  flipperMotor.reset(new rev::CANSparkMax(BALL_INTAKE_FLIP_MOTOR_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  intakeArmEnc.reset(new frc::Encoder(INTAKE_MOTOR_ENC1_DIO, INTAKE_MOTOR_ENC2_DIO));
  //intakeArmEnc->SetDistancePerPulse(INTAKE_ARM_TICK_TO_DEGREE_FACTOR);
  ballIntakeArmLimit.reset(new frc::DigitalInput(BALL_INTAKE_ARM_LIMIT_DIO));
  intakeFolderSolenoid.reset(new frc::Solenoid(BALL_INTAKE_FOLDER_PCM_ID));
  unfoldIntakeArms();
  ballIntakeMotor.reset(new ctre::phoenix::motorcontrol::can::WPI_VictorSPX(BALL_INTAKE_WHEELS_MOTOR_CAN_ID));
  intakeWheelPower = 0;
  
  // Setup Shuffleboard for each input and output device
  frc::ShuffleboardTab &shuffleTab = frc::Shuffleboard::GetTab("Ball Intake");

  frc::ShuffleboardLayout& shuffleList = shuffleTab.GetLayout("Flipper Motor", frc::BuiltInLayouts::kList);
  shuffleList.WithSize(1,3);
  nteFlipperMotor.setPower = (shuffleList.Add("Set Power", 0).GetEntry());
  nteFlipperMotor.outputCurrent = (shuffleList.Add("Current Out", 0).GetEntry());
  nteFlipperMotor.encoderPosition = (shuffleList.Add("Position", 0).GetEntry());
  nteFlipperMotor.encoderVelocity = (shuffleList.Add("Velocity", 0).GetEntry());
  nteFlipperMotor.motorTemperature = (shuffleList.Add("Motor Temp", 0).GetEntry());
  
  nteIntakeArmEncPosition = (shuffleTab.Add("Intake Pos", 0).GetEntry());
  nteBallIntakeArmLimit = (shuffleTab.Add("Intake Limit", 0).GetEntry());
  nteIntakeFolderSolenoid = (shuffleTab.Add("Folder", 0).GetEntry());
  nteBallIntakeMotor = (shuffleTab.Add("Intake", 0).GetEntry());

  // Initialize state machine
  intakeArmNewStateParameters = false;
  intakeArmNewControlMode = INTAKE_ARM_MODE_DISABLED;
  intakeArmNewMaxPower = 0.0;
  intakeArmNewTargetAngle = INTAKE_ARM_MIN_ANGLE;
  intakeArmNewState = INTAKE_ARM_STATE_IDLE;

  intakeArmControlMode = INTAKE_ARM_MODE_DISABLED;
  intakeArmMaxPower = 0.0;
  intakeArmTargetAngle = INTAKE_ARM_MIN_ANGLE;
  intakeArmState = INTAKE_ARM_STATE_IDLE;
  intakeArmLastPower = 0.0;
  intakeArmBlockedAngle = 0.0;
}

void BallIntakeSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new BallIntakeWithJoystickCmd());
}

void BallIntakeSub::updateShuffleBoard() {
  nteFlipperMotor.setPower.SetDouble(flipperMotor->Get());
  nteFlipperMotor.outputCurrent.SetDouble(flipperMotor->GetOutputCurrent());
  nteFlipperMotor.encoderPosition.SetDouble(getIntakeArmAngle());
  nteFlipperMotor.encoderVelocity.SetDouble(flipperMotor->GetEncoder().GetVelocity());
  nteFlipperMotor.motorTemperature.SetDouble(flipperMotor->GetMotorTemperature());
  
  nteIntakeArmEncPosition.SetDouble(intakeArmEnc->GetDistance());
  nteBallIntakeArmLimit.SetBoolean(ballIntakeArmLimit->Get());
  nteIntakeFolderSolenoid.SetBoolean(intakeFolderSolenoid->Get());
  nteBallIntakeMotor.SetDouble(ballIntakeMotor->Get());
}

void BallIntakeSub::setIntakeArmPower(double power) {
  flipperMotor->Set(-power);
}

double BallIntakeSub::getIntakeArmAngle() {
  return flipperMotor->GetEncoder().GetPosition() * INTAKE_ARM_TICK_TO_DEGREE_FACTOR;
}

double BallIntakeSub::getIntakeArmVelocity() {
  return intakeArmEnc->GetRate();
}

bool BallIntakeSub::isIntakeAtLimit() {
  return !ballIntakeArmLimit->Get();
}

void BallIntakeSub::unfoldIntakeArms() {
  intakeFolderSolenoid->Set(false);
}

void BallIntakeSub::foldIntakeArms() {
  intakeFolderSolenoid->Set(true);
}

bool BallIntakeSub::isIntakeUnfolded() {
  return !intakeFolderSolenoid->Get();
}

void BallIntakeSub::setIntakeWheelPower(double power) {
  ballIntakeMotor->Set(ControlMode::PercentOutput, power);
  intakeWheelPower = power;
}

double BallIntakeSub::getIntakeWheelPower() {
  return intakeWheelPower;
  // The Get function seems to be broken.
  //return ballIntakeMotor->Get();
}


void BallIntakeSub::setIntakeArmAngle(int mode, double maxPower, double targetAngle) {
  //only do something if the intake arm parameters have changed since last call
  if ((mode != intakeArmControlMode) || (maxPower != intakeArmMaxPower) || (targetAngle != intakeArmTargetAngle)) {
    // if an old input is pending, drop it
    intakeArmNewStateParameters = false;

    // Check input parameters
    if (fabs(maxPower) > 1.0) {
      logger.send(logger.BALLINTAKE, "SIA: ERROR!! Max power = %.2f \n", maxPower);
      return;
    }
    if ((mode != INTAKE_ARM_MODE_MANUAL) &&
      ((targetAngle < INTAKE_ARM_MIN_ANGLE) || (targetAngle > INTAKE_ARM_MAX_ANGLE))) {
      logger.send(logger.BALLINTAKE, "SIA: ERROR (#2)!! Angle = %.1f \n", targetAngle);
      return;
    }

    switch (mode) {
      case INTAKE_ARM_MODE_DISABLED:
        // Turn intake motors off
        intakeArmNewState = INTAKE_ARM_STATE_IDLE;
        intakeArmNewControlMode = mode;
        intakeArmNewMaxPower = 0.0;
        intakeArmNewTargetAngle = 0.0;
        intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
        logger.send(logger.BALLINTAKE, "SIA: Disabled\n");
        break;

      case INTAKE_ARM_MODE_AUTO:
        // Go to new target position
        intakeArmNewState = INTAKE_ARM_STATE_MOVING;
        intakeArmNewControlMode = mode;
        intakeArmNewMaxPower = fabs(maxPower);
        intakeArmNewTargetAngle = targetAngle;
        intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
        logger.send(logger.DEBUGGING, "SIA: Auto (P=%.2f, A=%.1f)\n", intakeArmNewMaxPower, intakeArmNewTargetAngle);
        break;

      case INTAKE_ARM_MODE_MANUAL:
        // Take the joystick input to determine the direction of travel and power
        // Note that in this mode the power is continually adjusted (i.e. this function gets called a lot).
        if (fabs(maxPower) < MANUAL_MODE_POWER_DEADBAND) {
          // Hold position
          if (intakeArmState != INTAKE_ARM_STATE_HOLDING) {
            intakeArmNewState = INTAKE_ARM_STATE_HOLDING;
            intakeArmNewControlMode = mode;
            intakeArmNewMaxPower = 0.0;
            intakeArmNewTargetAngle = getIntakeArmAngle();
            intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
            logger.send(logger.BALLINTAKE, "SIA: Man - deadzone start @ %.1f\n", intakeArmNewTargetAngle);
          }
          else {
          logger.send(logger.BALLINTAKE, "SFA: Holding \n");
          }
        }
        else {
          // Move using specified power
          intakeArmNewState = INTAKE_ARM_STATE_MOVING;
          intakeArmNewControlMode = mode;
          intakeArmNewMaxPower = fabs(maxPower);
          intakeArmNewTargetAngle = (maxPower > 0) ? INTAKE_ARM_MAX_ANGLE : INTAKE_ARM_MIN_ANGLE;
          intakeArmNewStateParameters = true; // Only set this to true after all the other parameters have been set
          logger.send(logger.BALLINTAKE, "SIA: Man - moving (P=%.2f, H=%.1f)\n", 
              intakeArmNewMaxPower, intakeArmNewTargetAngle);
        }
        break;
    }
  }
  else {
    logger.send(logger.BALLINTAKE, "SIA: No change (M=%d, P=%.2f, A=%.1f)\n", mode, maxPower, targetAngle);
  }
}

// Returns true if the arm angle is within tolerance of the target angle
bool BallIntakeSub::isIntakeArmAtTarget() {
  if (intakeArmNewStateParameters) {
    return false;
  }

  if ((fabs(intakeArmTargetAngle - getIntakeArmAngle()) < INTAKE_ARM_ANGLE_TOLERANCE) &&
      (fabs(getIntakeArmVelocity()) < INTAKE_ARM_VELOCITY_TOLERANCE)) {
    logger.send(logger.BALLINTAKE, "IIAAT: Arms at target (T=%.1f, C=%.1f, V=%.1f)\n", 
        intakeArmTargetAngle, getIntakeArmAngle(), getIntakeArmVelocity());
    return true;
  }
  else {
        //logger.send(logger.BALLINTAKE, "IIAAT: Arms not at target (T=%.1f, C=%.1f, V=%.1f)\n", 
    //    intakeArmTargetAngle, getIntakeArmAngle(), getIntakeArmVelocity());
    return false;
  }
}

void BallIntakeSub::updateIntakeArmStateMachine() {
  double newPower = 0.0;
  double currentAngle = getIntakeArmAngle();

  // Apply any new inputs
  if (intakeArmNewStateParameters) {
    intakeArmState = intakeArmNewState;
    intakeArmControlMode = intakeArmNewControlMode;
    intakeArmMaxPower = intakeArmNewMaxPower;
    intakeArmTargetAngle = intakeArmNewTargetAngle;
    intakeArmNewStateParameters = false;
    logger.send(logger.BALLINTAKE, "IASM: New     (P=%3.2f, S=%d, T=%6.1f, C=%6.1f, M=%d)\n", 
        intakeArmNewMaxPower, intakeArmState, intakeArmTargetAngle, currentAngle, intakeArmControlMode);
  }

  // In auto mode
  //  - The target angle and max power are set by the caller
  // In manual mode
  // - If the joystick is outside the dead zone, the target angle is set to the highest/lowest
  //   allowable angle with the max power based on the joystick value.
  // - If the max power is close to zero (in the dead zone), the target angle is set to the
  //   current angle.
  switch (intakeArmState) {
    case INTAKE_ARM_STATE_IDLE:
      // Don't drive motors, leave as is
      newPower = 0.0;
      break;

    case INTAKE_ARM_STATE_HOLDING:
      // Give the motor just enough power to keep the current position
  
      newPower = calcIntakeArmHoldPower(currentAngle, intakeArmTargetAngle);
      
      logger.send(logger.BALLINTAKE, "IASM: Holding (P=%3.4f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, intakeArmState, intakeArmTargetAngle, currentAngle);
      break;

    case INTAKE_ARM_STATE_MOVING:
      if (isIntakeArmBlocked(currentAngle, intakeArmTargetAngle)) {
        intakeArmBlockedAngle = currentAngle;
        intakeArmState = INTAKE_ARM_STATE_INTERRUPTED;
      }
      else if (isIntakeArmAtTarget()) {
        intakeArmState = INTAKE_ARM_STATE_HOLDING;
      }
      else {
        newPower = calcIntakeArmMovePower(currentAngle, intakeArmTargetAngle, intakeArmMaxPower);
      }
      logger.send(logger.BALLINTAKE, "IASM: Moving  (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, intakeArmState, intakeArmTargetAngle, currentAngle);
      break;

    case INTAKE_ARM_STATE_INTERRUPTED:
      if (!isIntakeArmBlocked(currentAngle, intakeArmTargetAngle)) {
        intakeArmState = INTAKE_ARM_STATE_MOVING;
      }
      else {
        newPower = calcIntakeArmHoldPower(currentAngle, intakeArmBlockedAngle);
      }
      logger.send(logger.BALLINTAKE, "IASM: Blocked (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, intakeArmState, intakeArmTargetAngle, currentAngle);
      break;

    default:
      // This should never happen.  Best thing we can do is hold our current position.
      logger.send(logger.ASSERTS, "Intake arm state machine entered invalid state (%d).  Fix the code!\n", intakeArmState);
      intakeArmState = INTAKE_ARM_STATE_HOLDING;
      break;
  }

  if (newPower != intakeArmLastPower) {
    setIntakeArmPower(newPower);
    intakeArmLastPower = newPower;
    //logger.send(logger.BALLINTAKE, "IASM: New power = %f (S=%d, M=%d, P=%.2f, H=%.1f)\n", newPower,
      // intakeArmState, intakeArmControlMode, intakeArmNewMaxPower, intakeArmNewTargetAngle);
  }
}

bool BallIntakeSub::isIntakeArmBlocked(double currentAngle, double targetAngle) {
  double direction = 1.0;

  if (currentAngle > targetAngle) {
    direction = -1.0;
  }
  isIntakeAtLimit();
  // TODO: implement all rules
  if (((currentAngle >= INTAKE_ARM_MAX_ANGLE) && (direction > 0)) ||
      ((currentAngle < INTAKE_ARM_MIN_ANGLE) && (direction < 0))/* || 
      (isIntakeAtLimit())*/) {
    return true;
  }

  return false;
}

double BallIntakeSub::calcIntakeArmHoldPower(double currentAngle, double targetAngle) {
  return  ((targetAngle - currentAngle) * 0.01) + (-0.02 / 90)*(targetAngle - 60);
}

double BallIntakeSub::calcIntakeArmMovePower(double currentAngle, double targetAngle, double maxPower) {
  double direction = 1.0;
  double newPower = 0;

  if (currentAngle > targetAngle) {
    direction = -1.0;
  }

  // Throttle the power once we get to 25 deg from the target
  double resultPower = (targetAngle - currentAngle) * 0.04;

  if (fabs(resultPower) > maxPower) {
    newPower = maxPower * direction;
  }
  else {
    newPower = resultPower;
  }

  return newPower;
}
