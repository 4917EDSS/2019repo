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
#include "subsystems/ElevatorSub.h"
#include "commands/ElevatorWithJoystickCmd.h"
#include "components/Log.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInLayouts.h>
#include "SparkShuffleboardEntrySet.h"
#include "Robot.h"

constexpr double ELEVATOR_POSITION_TOLERANCE_MM = 10.0;
constexpr double ELEVATOR_VELOCITY_TOLERANCE_MM_S = 45;
constexpr double MANUAL_MODE_POWER_DEADBAND = 0.03;
constexpr double ELEVATOR_TICK_TO_MM_FACTOR = (9.43);
constexpr double ELEVATOR_IS_DOWN_TOLERANCE_MM = ELEVATOR_POSITION_TOLERANCE_MM + 1.0;

// Elevator state machine states
constexpr int ELEVATOR_STATE_IDLE = 0;
constexpr int ELEVATOR_STATE_HOLDING = 1;
constexpr int ELEVATOR_STATE_MOVING = 2;
constexpr int ELEVATOR_STATE_INTERRUPTED = 3;

ElevatorSub::ElevatorSub() : Subsystem("ElevatorSub") {
  elevatorMotor1.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  elevatorMotor2.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  elevatorMotor1->GetEncoder().SetPosition(0); // Positive means rotating towards front
  elevatorMotor2->GetEncoder().SetPosition(0); // Not used but good to have as a backup
  elevatorMotor1->GetEncoder().SetPositionConversionFactor(ELEVATOR_TICK_TO_MM_FACTOR);
  elevatorMotor2->GetEncoder().SetPositionConversionFactor(ELEVATOR_TICK_TO_MM_FACTOR);
//limit switches
  lowerLimit.reset(new frc::DigitalInput(ELEVATOR_LOWER_LIMIT_DIO));
  upperLimit.reset(new frc::DigitalInput(ELEVATOR_UPPER_LIMIT_DIO));
//shifter
  shifterSolenoid.reset(new frc::Solenoid(CLIMB_GEAR_PCM_ID));
  setShifterHigh(false);

  // Setup Shuffleboard for each input and output device
  frc::ShuffleboardTab &shuffleTab = frc::Shuffleboard::GetTab("Elevator");

  for (int index = 0; index < 2; index++) {
    std::string listName = "Motor " + std::to_string(index) + " Data";
    frc::ShuffleboardLayout &shuffleList = shuffleTab.GetLayout(listName, frc::BuiltInLayouts::kList);
    shuffleList.WithSize(1, 3);
    shuffleList.WithPosition(index, 0);
    nteSparksTwo[index].setPower = (shuffleList.Add("Set Power", 0).GetEntry());
    nteSparksTwo[index].outputCurrent = (shuffleList.Add("Current Out", 0).GetEntry());
    nteSparksTwo[index].encoderPosition = (shuffleList.Add("Position", 0).GetEntry());
    nteSparksTwo[index].encoderVelocity = (shuffleList.Add("Velocity", 0).GetEntry());
    nteSparksTwo[index].motorTemperature = (shuffleList.Add("Motor Temp", 0).GetEntry());
  }
  nteShifterSolenoid = (shuffleTab.Add("Shiffter", 0).GetEntry());

  frc::ShuffleboardLayout &shuffleList = shuffleTab.GetLayout("State Machine", frc::BuiltInLayouts::kList);
  shuffleList.WithSize(1, 4);
  shuffleList.WithPosition(3, 0);
  nteSmMode = (shuffleList.Add("Mode", 0).GetEntry());
  nteSmState = (shuffleList.Add("State", 0).GetEntry());
  nteLastPower = (shuffleList.Add("Power", 0).GetEntry());
  nteSmMaxPower = (shuffleList.Add("Max Power", 0).GetEntry());
  nteSmTarget = (shuffleList.Add("Target", 0).GetEntry());
  nteSmBlockedAt = (shuffleList.Add("Blocked At", 0).GetEntry());
  nteSmIsFinished = (shuffleList.Add("Is Finished", 0).GetEntry());

  // Initialize elevator state machine
  elevatorNewStateParameters = false;
  elevatorNewControlMode = ELEVATOR_MODE_DISABLED;
  elevatorNewMaxPower = 0.0;
  elevatorNewTargetHeightMm = ELEVATOR_MIN_HEIGHT_MM;
  elevatorNewState = ELEVATOR_STATE_IDLE;

  elevatorControlMode = ELEVATOR_MODE_DISABLED;
  elevatorMaxPower = 0.0;
  elevatorTargetHeightMm = ELEVATOR_MIN_HEIGHT_MM;
  elevatorState = ELEVATOR_STATE_IDLE;
  elevatorLastPower = 0.0;
  elevatorBlockedHeightMm = 0.0;
}

void ElevatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new ElevatorWithJoystickCmd());
}

void ElevatorSub::updateShuffleBoard() {
  nteSparksTwo[0].setPower.SetDouble(elevatorMotor1->Get());
  nteSparksTwo[0].outputCurrent.SetDouble(elevatorMotor1->GetOutputCurrent());
  nteSparksTwo[0].encoderPosition.SetDouble(elevatorMotor1->GetEncoder().GetPosition()) + ELEVATOR_MIN_HEIGHT_MM;
  nteSparksTwo[0].encoderVelocity.SetDouble(elevatorMotor1->GetEncoder().GetVelocity());
  nteSparksTwo[0].motorTemperature.SetDouble(elevatorMotor1->GetMotorTemperature());

  nteSparksTwo[1].setPower.SetDouble(elevatorMotor2->Get());
  nteSparksTwo[1].outputCurrent.SetDouble(elevatorMotor2->GetOutputCurrent());
  nteSparksTwo[1].encoderPosition.SetDouble(elevatorMotor2->GetEncoder().GetPosition() + ELEVATOR_MIN_HEIGHT_MM);
  nteSparksTwo[1].encoderVelocity.SetDouble(elevatorMotor2->GetEncoder().GetVelocity());
  nteSparksTwo[1].motorTemperature.SetDouble(elevatorMotor2->GetMotorTemperature());

  nteShifterSolenoid.SetBoolean(shifterSolenoid->Get());

  nteSmMode.SetDouble(elevatorControlMode);
  nteSmState.SetDouble(elevatorState);
  nteLastPower.SetDouble(elevatorLastPower);
  nteSmMaxPower.SetDouble(elevatorMaxPower);
  nteSmTarget.SetDouble(elevatorTargetHeightMm);
  nteSmBlockedAt.SetDouble(elevatorBlockedHeightMm);
  nteSmIsFinished.SetBoolean(isElevatorAtTarget());
}

void ElevatorSub::setElevatorMotorPower(double power) {
  elevatorMotor1->Set(-power);
  elevatorMotor2->Set(power);
}

double ElevatorSub::getElevatorHeight() {
  return elevatorMotor2->GetEncoder().GetPosition() + ELEVATOR_MIN_HEIGHT_MM;
}

double ElevatorSub::getElevatorVelocity() {
  return elevatorMotor2->GetEncoder().GetVelocity();
}

bool ElevatorSub::isElevatorDown() {
  if ((getElevatorHeight() - ELEVATOR_MIN_HEIGHT_MM) < ELEVATOR_IS_DOWN_TOLERANCE_MM) {
    return true;
  }
  else {
    return false;
  }
}

// Set true for High Gear, false for Low Gear
void ElevatorSub::setShifterHigh(bool highGear) {
  shifterSolenoid->Set(highGear);
}

bool ElevatorSub::isShifterHigh() {
  return shifterSolenoid->Get();
}

// Set the elevator mode and height
// - ELEVATOR_MODE_DISABLED turns off elevator motor (maxPower and targetHeightMm are ignored)
// - ELEVATOR_MODE_AUTO has the state machine move the elevator to the desire height
// - ELEVATOR_MODE_MANUAL is for joystick input (targetHeightMm is ignored)
void ElevatorSub::setElevatorHeight(int mode, double maxPower, double targetHeightMm) {
  // Only do something if one of the parameters has changed
  if ((mode != elevatorControlMode) || (maxPower != elevatorMaxPower) || (targetHeightMm != elevatorTargetHeightMm)) {
    // If an old input is pending, drop it
    elevatorNewStateParameters = false;

    // Check input parameters
    if (fabs(maxPower) > 1.0) {
      logger.send(logger.ELEVATOR, "ESH: ERROR!! Max power = %.2f \n", maxPower);
      return;
    }
    if ((mode != ELEVATOR_MODE_MANUAL) && 
        (targetHeightMm < ELEVATOR_MIN_HEIGHT_MM) || (targetHeightMm > ELEVATOR_MAX_HEIGHT_MM)) {
      logger.send(logger.ELEVATOR, "ESH: ERROR (#2)!! Height = %.1f \n", targetHeightMm);
      return;
    }

    switch (mode) {
      case ELEVATOR_MODE_DISABLED:
        // Turn elevator motors off
        elevatorNewState = ELEVATOR_STATE_IDLE;
        elevatorNewControlMode = mode;
        elevatorNewMaxPower = 0.0;
        elevatorNewTargetHeightMm = ELEVATOR_MIN_HEIGHT_MM;
        elevatorNewStateParameters = true; // Only set this to true after all the other parameters have been set
        logger.send(logger.ELEVATOR, "ESH: Disabled\n");
        break;

      case ELEVATOR_MODE_AUTO:
        // Go to new target position
        elevatorNewState = ELEVATOR_STATE_MOVING;
        elevatorNewControlMode = mode;
        elevatorNewMaxPower = fabs(maxPower);
        elevatorNewTargetHeightMm = targetHeightMm;
        elevatorNewStateParameters = true; // Only set this to true after all the other parameters have been set
        logger.send(logger.ELEVATOR, "ESH: Auto (P=%.2f, H=%.1f)\n", elevatorNewMaxPower, elevatorNewTargetHeightMm);
        break;

    case ELEVATOR_MODE_MANUAL:
      // Take the joystick input to determine the direction of travel and power
      // Note that in this mode the power is continually adjusted (i.e. this function gets called a lot).
      if (fabs(maxPower) < MANUAL_MODE_POWER_DEADBAND) {
        // Hold position
        if (elevatorState != ELEVATOR_STATE_HOLDING) {
          elevatorNewState = ELEVATOR_STATE_HOLDING;
          elevatorNewControlMode = mode;
          elevatorNewMaxPower = 0.0;
          elevatorNewTargetHeightMm = getElevatorHeight();
          elevatorNewStateParameters = true; // Only set this to true after all the other parameters have been set
          logger.send(logger.ELEVATOR, "ESH: Man - deadzone start @ %.1f\n", elevatorNewTargetHeightMm);
        }
        else {
          //logger.send(logger.ELEVATOR, "ESH: Holding \n");
        }
      }
      else {
        // Move using specified power
        elevatorNewState = ELEVATOR_STATE_MOVING;
        elevatorNewControlMode = mode;
        elevatorNewMaxPower = fabs(maxPower);
        elevatorNewTargetHeightMm = (maxPower > 0) ? ELEVATOR_MAX_HEIGHT_MM : ELEVATOR_MIN_HEIGHT_MM;
        elevatorNewStateParameters = true; // Only set this to true after all the other parameters have been set
        logger.send(logger.ELEVATOR, "ESH: Man - moving (P=%.2f, H=%.1f)\n", 
            elevatorNewMaxPower, elevatorNewTargetHeightMm);
      }
      break;
    }
  }
  else {
    logger.send(logger.ELEVATOR, "ESH: No change (M=%d, P=%.2f, H=%.1f)\n", mode, maxPower, targetHeightMm);
  }
}

// Returns true if the elevator is within tolerance of the target height
bool ElevatorSub::isElevatorAtTarget() {
  if (elevatorNewStateParameters) {
    // Haven't even implemented the new request so we can't be done
    return false;
  }

  if ((fabs(elevatorTargetHeightMm - getElevatorHeight()) < ELEVATOR_POSITION_TOLERANCE_MM) &&
      (fabs(getElevatorVelocity()) < ELEVATOR_VELOCITY_TOLERANCE_MM_S)) {
    logger.send(logger.ELEVATOR, "IEAT: Elevator at target (T=%.1f, C=%.1f, V=%.1f)\n", 
        elevatorTargetHeightMm, getElevatorHeight(), getElevatorVelocity());
    return true;
  }
  else {
  // logger.send(logger.ELEVATOR, "IEAT: Elevator not at target (T=%.1f, C=%.1f, V=%.1f)\n", 
  //      elevatorTargetHeightMm, getElevatorHeight(), getElevatorVelocity());
    return false;
  }
}

// This is the state machine that manages the elevator's motion.  It should be called on at a regular time interval
void ElevatorSub::updateElevatorStateMachine() {
  double newPower = 0.0;
  double currentHeightMm = getElevatorHeight();

  // Apply any new inputs
  if (elevatorNewStateParameters) {
    elevatorState = elevatorNewState;
    elevatorControlMode = elevatorNewControlMode;
    elevatorMaxPower = elevatorNewMaxPower;
    elevatorTargetHeightMm = elevatorNewTargetHeightMm;
    elevatorNewStateParameters = false;
    logger.send(logger.ELEVATOR, "ESM: New     (P=%3.2f, S=%d, T=%6.1f, C=%6.1f, M=%d)\n", 
        elevatorNewMaxPower, elevatorState, elevatorTargetHeightMm, currentHeightMm, elevatorControlMode);
  }

  // In auto mode
  //  - The target height and max power are set by the caller
  // In manual mode
  // - If the joystick is outside the dead zone, the target height is set to the highest/lowest
  //   allowable height with the max power based on the joystick value.
  // - If the max power is close to zero (in the dead zone), the target height is set to the
  //   current height.
  switch (elevatorState) {
    case ELEVATOR_STATE_IDLE:
      // Don't drive motors, leave as is
      newPower = 0.0;
      break;

    case ELEVATOR_STATE_HOLDING:
      // Give the motor just enough power to keep the current position
      newPower = calcElevatorHoldPower(currentHeightMm, elevatorTargetHeightMm);
      logger.send(logger.ELEVATOR, "ESM: Holding (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, elevatorState, elevatorTargetHeightMm, currentHeightMm);
      break;

    case ELEVATOR_STATE_MOVING:
      if (isElevatorBlocked(currentHeightMm, elevatorTargetHeightMm)) {
        elevatorBlockedHeightMm = currentHeightMm;
        elevatorState = ELEVATOR_STATE_INTERRUPTED;
      }
      else if (isElevatorAtTarget()) {
        elevatorState = ELEVATOR_STATE_HOLDING;
      }
      else {
        newPower = calcElevatorMovePower(currentHeightMm, elevatorTargetHeightMm, elevatorMaxPower);
      }
      logger.send(logger.ELEVATOR, "ESM: Moving  (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, elevatorState, elevatorTargetHeightMm, currentHeightMm);
      break;

    case ELEVATOR_STATE_INTERRUPTED:
      if (!isElevatorBlocked(currentHeightMm, elevatorTargetHeightMm)) {
        elevatorState = ELEVATOR_STATE_MOVING;
      }
      else {
        newPower = calcElevatorHoldPower(currentHeightMm, elevatorBlockedHeightMm);
      }
      logger.send(logger.ELEVATOR, "ESM: Blocked (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, elevatorState, elevatorBlockedHeightMm, currentHeightMm);
      break;

    default:
      // This should never happen.  Best thing we can do is hold our current position.
      logger.send(logger.ASSERTS, "Elevator state machine entered invalid state (%d).  Fix the code!\n", elevatorState);
      elevatorState = ELEVATOR_STATE_HOLDING;
      break;
  }

  if (newPower != elevatorLastPower) {
    setElevatorMotorPower(newPower);
    elevatorLastPower = newPower;
    //logger.send(logger.ELEVATOR, "ESM: New power = %f (S=%d, M=%d, P=%.2f, H=%.1f)\n", newPower,
    //    elevatorState, elevatorControlMode, elevatorMaxPower, elevatorTargetHeightMm);
  }
}

bool ElevatorSub::isElevatorBlocked(double currentHeightMm, double targetHeightMm) {
  double direction = 1.0;

  if (currentHeightMm > targetHeightMm) {
    direction = -1.0;
  }

//  if (lowerLimit->Get() && direction < 0) {
//    return true;
//  }

  if (((currentHeightMm >= ELEVATOR_MAX_HEIGHT_MM) && (direction > 0)) ||
      ((currentHeightMm < ELEVATOR_MIN_HEIGHT_MM) && (direction < 0))) {
    return true;
  }

  // See if Manipulator is blocking movement
  if (direction > 0) {
    if ((Robot::manipulatorSub.getFlipperAngle()) < 30 && (Robot::manipulatorSub.getFlipperAngle() > -45)) {
      // Can only move up a bit if the manipulator is close to vertical
      if (currentHeightMm >= (ELEVATOR_MIN_HEIGHT_MM + 120)) {
        return true;
      }
    }

    if (Robot::manipulatorSub.getFlipperAngle() <= -45) {
      if (currentHeightMm >= (ELEVATOR_MIN_HEIGHT_MM + 500)) {
        return true;
      }
    }
  }
  
  return false;
}

double ElevatorSub::calcElevatorHoldPower(double currentHeightMm, double targetHeightMm) {
  double holdPower = (targetHeightMm - currentHeightMm) * 0.004;

  return holdPower;
}

double ElevatorSub::calcElevatorMovePower(double currentHeightMm, double targetHeightMm, double maxElevatorPower) {
  double direction = 1.0;
  double newPower = 0;

  if (currentHeightMm > targetHeightMm) {
    direction = -1.0;
  }

  // Throttle the power once we get to 100mm from the target
  double resultPower = (targetHeightMm - currentHeightMm) * 0.01;

  if(fabs(resultPower) > maxElevatorPower) {
    newPower = maxElevatorPower * direction;
  }
  else {
    newPower = resultPower;
  }

  return newPower;
}
