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
#include "subsystems/ElevatorSub.h"
#include "components/Log.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/BuiltInLayouts.h>
#include "SparkShuffleboardEntrySet.h"
#include "commands/ManipulatorWithJoystickCmd.h"
#include <Robot.h>

constexpr double FLIPPER_ANGLE_TOLERANCE = 1.0;
constexpr double FLIPPER_VELOCITY_TOLERANCE = 45;
constexpr double FLIPPER_TICK_TO_DEGREE_FACTOR = (90/40.1900);
constexpr double MANUAL_MODE_POWER_DEADBAND = 0.03;

// Manipulator state machine states
constexpr int FLIPPER_STATE_IDLE = 0;
constexpr int FLIPPER_STATE_HOLDING = 1;
constexpr int FLIPPER_STATE_MOVING = 2;
constexpr int FLIPPER_STATE_INTERRUPTED = 3;

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
  shuffleList.WithSize(1,3);
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

  // Initialize state machine
  flipperNewStateParameters = false;
  flipperNewControlMode = FLIPPER_MODE_DISABLED;
  flipperNewMaxPower = 0.0;
  flipperNewTargetAngle = MANIPULATOR_MIN_ANGLE;
  flipperNewState = FLIPPER_STATE_IDLE;

  flipperControlMode = FLIPPER_MODE_DISABLED;
  flipperMaxPower = 0.0;
  flipperTargetAngle = MANIPULATOR_MIN_ANGLE;
  flipperState = FLIPPER_STATE_IDLE;
  flipperLastPower = 0.0;
  flipperBlockedAngle = 0.0;
}

void ManipulatorSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
  SetDefaultCommand(new ManipulatorWithJoystickCmd());
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

double ManipulatorSub::getFlipperTargetAngle() {
  return flipperTargetAngle;
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
  intakeMotorRight->Set(-power);
  intakeMotorLeft->Set(power);
}

double ManipulatorSub::getIntakePower() {
  return intakeMotorLeft->Get();
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

// Set the flipper mode and angle
// - FLIPPER_MODE_DISABLED turns off flipper motor (maxPower and targetAngle are ignored)
// - FLIPPER_MODE_AUTO has the state machine move the flipper to the desire angle
// - FLIPPER_MODE_MANUAL is for joystick input (targetAngle is ignored)
void ManipulatorSub::setFlipperAngle(int mode, double maxPower, double targetAngle) {
  logger.send(logger.MANIPULATOR, "SFA: Start (P=%.2f, A=%.1f)\n", maxPower, targetAngle);     
  
  // Only do something if one of the parameters has changed
  if((mode != flipperControlMode) || (maxPower != flipperMaxPower) || (targetAngle != flipperTargetAngle)) {
    // If an old input is pending, drop it
    flipperNewStateParameters = false; 

    // Check input parameters
    if (fabs(maxPower) > 1.0) {
      logger.send(logger.MANIPULATOR, "SFA: ERROR!! Max power = %.2f \n", maxPower);
      return; 
    }
    if ((mode != FLIPPER_MODE_MANUAL) && 
        ((targetAngle < MANIPULATOR_MIN_ANGLE) || (targetAngle > MANIPULATOR_MAX_ANGLE))) {
      logger.send(logger.MANIPULATOR, "SFA: ERROR (#2)!! Angle = %.1f \n", targetAngle);
      return;
    }
    
    switch(mode) {
      case FLIPPER_MODE_DISABLED:
        // Turn flipper motors off
        flipperNewState = FLIPPER_STATE_IDLE;
        flipperNewControlMode = mode;
        flipperNewMaxPower = 0.0;
        flipperNewTargetAngle = 0.0;
        flipperNewStateParameters = true;    // Only set this to true after all the other parameters have been set
        logger.send(logger.MANIPULATOR, "SFA: Disabled\n");
        break;

      case FLIPPER_MODE_AUTO:
        // Go to new target position
        flipperNewState = FLIPPER_STATE_MOVING;
        flipperNewControlMode = mode;
        flipperNewMaxPower = fabs(maxPower);
        flipperNewTargetAngle = targetAngle;
        flipperNewStateParameters = true;    // Only set this to true after all the other parameters have been set
        logger.send(logger.MANIPULATOR, "SFA: Auto (P=%.2f, A=%.1f)\n", flipperNewMaxPower, flipperNewTargetAngle);
        break;

      case FLIPPER_MODE_MANUAL:
        // Take the joystick input to determine the direction of travel and power
        // Note that in this mode the power is continually adjusted (i.e. this function gets called a lot).
        if(fabs(maxPower) < MANUAL_MODE_POWER_DEADBAND) {
          // Hold position
          if(flipperState != FLIPPER_STATE_HOLDING) {
            flipperNewState = FLIPPER_STATE_HOLDING;
            flipperNewControlMode = mode;
            flipperNewMaxPower = 0.0;
            flipperNewTargetAngle = getFlipperAngle();
            flipperNewStateParameters = true;    // Only set this to true after all the other parameters have been set
            logger.send(logger.MANIPULATOR, "SFA: Man - deadzone start @ %.1f\n", flipperNewTargetAngle);
          }
          else {
          //logger.send(logger.MANIPULATOR, "SFA: Holding \n");
          }
        } 
        else {
          // Move using specified power
          flipperNewState = FLIPPER_STATE_MOVING;
          flipperNewControlMode = mode;
          flipperNewMaxPower = fabs(maxPower);;
          flipperNewTargetAngle = (maxPower > 0) ? MANIPULATOR_MAX_ANGLE : MANIPULATOR_MIN_ANGLE;
          flipperNewStateParameters = true;    // Only set this to true after all the other parameters have been set 
          logger.send(logger.MANIPULATOR, "SFA: Man - moving (P=%.2f, H=%.1f)\n", 
              flipperNewMaxPower, flipperNewTargetAngle);
        }
        break;
    }
  }
  else {
    logger.send(logger.MANIPULATOR, "SFA: No change (M=%d, P=%.2f, A=%.1f)\n", mode, maxPower, targetAngle);
  }
}

// Returns true if the flipper is within tolerance of the target angle
bool ManipulatorSub::isFlipperAtTarget() {
  if (flipperNewStateParameters) {
    // Haven't even implemented the new request so we can't be done
    return false;
  }

  if ((fabs(flipperTargetAngle - getFlipperAngle()) < FLIPPER_ANGLE_TOLERANCE) && 
      (fabs(getFlipperVelocity()) < FLIPPER_VELOCITY_TOLERANCE)) {
      logger.send(logger.MANIPULATOR, "IFAT: Flipper at target (T=%.1f, C=%.1f, V=%.1f)\n", 
        flipperTargetAngle, getFlipperAngle(), getFlipperVelocity());
      return true;
  } else {
    //logger.send(logger.MANIPULATOR, "IFAT: Flipper not at target (T=%.1f, C=%.1f, V=%.1f)\n", 
    //    flipperTargetAngle, getFlipperAngle(), getFlipperVelocity());
    return false;
  }
}

// This is the state machine that manages the flipper's motion.  It should be called on at a regular time interval.
void ManipulatorSub::updateFlipperStateMachine() {
  double newPower = 0.0;
  double currentAngle = getFlipperAngle();

  // Apply any new inputs
  if(flipperNewStateParameters) {
    flipperState = flipperNewState;
    flipperControlMode = flipperNewControlMode;
    flipperMaxPower = flipperNewMaxPower;
    flipperTargetAngle = flipperNewTargetAngle;
    flipperNewStateParameters = false;
    logger.send(logger.MANIPULATOR, "FSM: New     (P=%3.2f, S=%d, T=%6.1f, C=%6.1f, M=%d)\n", 
        flipperNewMaxPower, flipperState, flipperTargetAngle, currentAngle, flipperControlMode);
  }

  // In auto mode
  //  - The target angle and max power are set by the caller
  // In manual mode
  // - If the joystick is outside the dead zone, the target angle is set to the highest/lowest 
  //   allowable angle with the max power based on the joystick value.  
  // - If the max power is close to zero (in the dead zone), the target angle is set to the 
  //   current angle.
  switch(flipperState) {
    case FLIPPER_STATE_IDLE:
      // Don't drive motors, leave as is
      newPower = 0.0;
      break;

    case FLIPPER_STATE_HOLDING:
      // Give the motor just enough power to keep the current position
      newPower = calcFlipperHoldPower(currentAngle, flipperTargetAngle);
      logger.send(logger.MANIPULATOR, "FSM: Holding (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, flipperState, flipperTargetAngle, currentAngle);
      break;

    case FLIPPER_STATE_MOVING:
      if(isFlipperBlocked(currentAngle, flipperTargetAngle)) {
        flipperBlockedAngle = currentAngle;
        flipperState = FLIPPER_STATE_INTERRUPTED;
      }
      else if(isFlipperAtTarget()) {
        flipperState = FLIPPER_STATE_HOLDING;
      }
      else {
        newPower = calcFlipperMovePower(currentAngle, flipperTargetAngle, flipperMaxPower);
      }
      logger.send(logger.MANIPULATOR, "FSM: Moving  (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, flipperState, flipperTargetAngle, currentAngle);
      break;

    case FLIPPER_STATE_INTERRUPTED:
      if(!isFlipperBlocked(currentAngle, flipperTargetAngle)) {
        flipperState = FLIPPER_STATE_MOVING;
      }
      else {
         newPower = calcFlipperHoldPower(currentAngle, flipperBlockedAngle);
      }
      logger.send(logger.MANIPULATOR, "FSM: Blocked (P=%3.2f, S=%d, T=%6.1f, C=%6.1f)\n",
          newPower, flipperState, flipperBlockedAngle, currentAngle);
      break;

    default:
      // This should never happen.  Best thing we can do is hold our current position.
      logger.send(logger.ASSERTS, "Flipper state machine entered invalid state (%d).  Fix the code!\n", flipperState);
      flipperState = FLIPPER_STATE_HOLDING;
      break;
  }

  if(newPower != flipperLastPower) {
    setFlipperPower(newPower);
    flipperLastPower = newPower;
    logger.send(logger.MANIPULATOR, "FSM: New power = %f (S=%d, M=%d, P=%.2f, H=%.1f)\n", newPower,
        flipperState, flipperControlMode, flipperNewMaxPower, flipperNewTargetAngle);
  }
}

bool ManipulatorSub::isFlipperBlocked(double currentAngle, double targetAngle) {
  double direction = 1.0;
  
  if(currentAngle > targetAngle) {
    direction = -1.0;
  }

  // TODO: implement all rules
  if(((currentAngle >= MANIPULATOR_MAX_ANGLE) && (direction > 0)) || 
      ((currentAngle < MANIPULATOR_MIN_ANGLE) && (direction < 0))) {
    return true;
  }
  
  if (Robot::elevatorSub.getElevatorHeight() >= (ELEVATOR_MIN_HEIGHT_MM + 120)) {
    if (((currentAngle > -90) && (currentAngle < 0)) && (direction < 0)) {
      return true;
    }
    if (((currentAngle > 0) && (currentAngle < 30)) && (direction < 0)) {
      return true;
    }
  }

  return false;
}

double ManipulatorSub::calcFlipperHoldPower(double currentAngle, double targetAngle) {
  // 3% power holds flipper at a 90 degree angle  
  // Make propertional to target.
  return ((0.025 / 90) * (-targetAngle)) + ((targetAngle - currentAngle) * 0.001);
}

double ManipulatorSub::calcFlipperMovePower(double currentAngle, double targetAngle, double maxPower) {
  double direction = 1.0;
  double newPower = 0;
  
  if(currentAngle > targetAngle) {
    direction = -1.0;
  }

  // Throttle the power once we get to 25 deg from the target
  double resultPower = (targetAngle - currentAngle) * 0.04;

  if(fabs(resultPower) > maxPower) {
    newPower = maxPower * direction;
  }
  else {
    newPower = resultPower;
  }
  
  return newPower;
}
