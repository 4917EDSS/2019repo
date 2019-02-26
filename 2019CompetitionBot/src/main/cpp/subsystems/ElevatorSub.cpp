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


constexpr double ELEVATOR_POSITION_TOLERANCE_MM = 5.0;
constexpr double ELEVATOR_VELOCITY_TOLERANCE_MM_S = 45;               // TODO Determine value
constexpr double MANUAL_MODE_POWER_DEADBAND = 0.03;
constexpr double ELEVATOR_TICK_TO_MM_FACTOR = (28.94679);             // TODO Determine value
constexpr double ELEVATOR_IS_DOWN_TOLERANCE_MM = ELEVATOR_POSITION_TOLERANCE_MM + 1.0;

// Elevator state machine states
constexpr int ELEVATOR_STATE_IDLE = 0;
constexpr int ELEVATOR_STATE_HOLDING = 1;
constexpr int ELEVATOR_STATE_MOVING = 2;
constexpr int ELEVATOR_STATE_INTERRUPTED = 3;


ElevatorSub::ElevatorSub() : Subsystem("ElevatorSub") {
  elevatorMotor1.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_1_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  elevatorMotor2.reset(new rev::CANSparkMax(ELEVATOR_MOTOR_2_CAN_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless));
  elevatorMotor1->GetEncoder().SetPosition(0);  // Positive means rotating towards front
  elevatorMotor2->GetEncoder().SetPosition(0);  // Not used but good to have as a backup
  elevatorMotor1->GetEncoder().SetPositionConversionFactor(ELEVATOR_TICK_TO_MM_FACTOR);
  elevatorMotor2->GetEncoder().SetPositionConversionFactor(ELEVATOR_TICK_TO_MM_FACTOR);

  // TODO: Add limit switch

  shifterSolenoid.reset(new frc::Solenoid(CLIMB_GEAR_PCM_ID));
  setShifterHigh(true);

  // Setup Shuffleboard for each input and output device
  frc::ShuffleboardTab& shuffleTab = frc::Shuffleboard::GetTab("Elevator");

  for (int index = 0; index < 2; index++){
    std::string listName = "Motor " + std::to_string(index) + " Data";
    frc::ShuffleboardLayout& shuffleList = shuffleTab.GetLayout(listName, frc::BuiltInLayouts::kList);
    shuffleList.WithSize(1,3);
    shuffleList.WithPosition(index,0);
    nteSparksTwo[index].setPower = (shuffleList.Add("Set Power", 0).GetEntry());
    nteSparksTwo[index].outputCurrent = (shuffleList.Add("Current Out", 0).GetEntry());
    nteSparksTwo[index].encoderPosition = (shuffleList.Add("Position", 0).GetEntry());
    nteSparksTwo[index].encoderVelocity = (shuffleList.Add("Velocity", 0).GetEntry());
    nteSparksTwo[index].motorTemperature = (shuffleList.Add("Motor Temp", 0).GetEntry());
  }
  nteShifterSolenoid = (shuffleTab.Add("Shiffter", 0).GetEntry());

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

void ElevatorSub::updateShuffleBoard(){
  nteSparksTwo[0].setPower.SetDouble(elevatorMotor1->Get());
  nteSparksTwo[0].outputCurrent.SetDouble(elevatorMotor1->GetOutputCurrent());
  nteSparksTwo[0].encoderPosition.SetDouble(elevatorMotor1->GetEncoder().GetPosition());
  nteSparksTwo[0].encoderVelocity.SetDouble(elevatorMotor1->GetEncoder().GetVelocity());
  nteSparksTwo[0].motorTemperature.SetDouble(elevatorMotor1->GetMotorTemperature());

  nteSparksTwo[1].setPower.SetDouble(elevatorMotor2->Get());
  nteSparksTwo[1].outputCurrent.SetDouble(elevatorMotor2->GetOutputCurrent());
  nteSparksTwo[1].encoderPosition.SetDouble(elevatorMotor2->GetEncoder().GetPosition());
  nteSparksTwo[1].encoderVelocity.SetDouble(elevatorMotor2->GetEncoder().GetVelocity());
  nteSparksTwo[1].motorTemperature.SetDouble(elevatorMotor2->GetMotorTemperature());

  nteShifterSolenoid.SetBoolean(shifterSolenoid->Get());
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
  if((getElevatorHeight() - ELEVATOR_MIN_HEIGHT_MM) < ELEVATOR_IS_DOWN_TOLERANCE_MM) {
    return true;
  }
  else {
    return false;
  }
}

// Set true for High Gear, false for Low Gear
void ElevatorSub::setShifterHigh(bool highGear){
  shifterSolenoid->Set(highGear);
}

bool ElevatorSub::isShifterHigh(){
  return shifterSolenoid->Get();
}

// Set the elevator mode and height
// - ELEVATOR_MODE_DISABLED turns off elevator motor (maxPower and targetHeightMm are ignored)
// - ELEVATOR_MODE_AUTO has the state machine move the elevator to the desire height
// - ELEVATOR_MODE_MANUAL is for joystick input (targetHeightMm is ignored)
void ElevatorSub::setElevatorHeight(int mode, double maxPower, double targetHeightMm) {
  logger.send(logger.ELEVATOR, "ESH: START: (P=%.2f, H=%.1f)\n", maxPower, targetHeightMm);     
  
  // Only do something if one of the parameters has changed
  if((mode != elevatorControlMode) || (maxPower != elevatorMaxPower) || (targetHeightMm != elevatorTargetHeightMm)) {
    // If an old input is pending, drop it
    elevatorNewStateParameters = false;

    // Check input parameters
    if (fabs(maxPower) > 1.0) {
      return; 
    }
    if ((targetHeightMm < ELEVATOR_MIN_HEIGHT_MM) || (targetHeightMm > ELEVATOR_MAX_HEIGHT_MM)) {
      return;
    }
    
    switch(mode) {
      case ELEVATOR_MODE_DISABLED:
        // Turn elevator motors off
        elevatorNewState = ELEVATOR_STATE_IDLE;
        elevatorNewControlMode = mode;
        elevatorNewMaxPower = 0.0;
        elevatorNewTargetHeightMm = ELEVATOR_MIN_HEIGHT_MM;
        elevatorNewStateParameters = true;    // Only set this to true after all the other parameters have been set
        logger.send(logger.ELEVATOR, "ESH: Disabled\n");
        break;

      case ELEVATOR_MODE_AUTO:
        // Go to new target position
        elevatorNewState = ELEVATOR_STATE_MOVING;
        elevatorNewControlMode = mode;
        elevatorNewMaxPower = fabs(maxPower);
        elevatorNewTargetHeightMm = targetHeightMm;
        elevatorNewStateParameters = true;    // Only set this to true after all the other parameters have been set
        logger.send(logger.ELEVATOR, "ESH: Auto (P=%.2f, H=%.1f)\n", elevatorNewMaxPower, elevatorNewTargetHeightMm);
        break;

      case ELEVATOR_MODE_MANUAL:
        // Take the joystick input to determine the direction of travel and power
        // Note that in this mode the power is continually adjusted (i.e. this function gets called a lot).
        if(fabs(maxPower) < MANUAL_MODE_POWER_DEADBAND) {
          // Hold position
          if(elevatorState != ELEVATOR_STATE_HOLDING) {
            elevatorNewState = ELEVATOR_STATE_HOLDING;
            elevatorNewControlMode = mode;
            elevatorNewMaxPower = 0.0;
            elevatorNewTargetHeightMm = getElevatorHeight();
            elevatorNewStateParameters = true;    // Only set this to true after all the other parameters have been set
            logger.send(logger.ELEVATOR, "ESH: Man - deadzone start @ %f\n", elevatorNewTargetHeightMm);
          }
        } 
        else {
          // Move using specified power
          elevatorNewState = ELEVATOR_STATE_MOVING;
          elevatorNewControlMode = mode;
          elevatorNewMaxPower = fabs(maxPower);;
          elevatorNewTargetHeightMm = (maxPower > 0) ? ELEVATOR_MAX_HEIGHT_MM : ELEVATOR_MIN_HEIGHT_MM;
          elevatorNewStateParameters = true;    // Only set this to true after all the other parameters have been set 
          logger.send(logger.ELEVATOR, "ESH: Man - moving (P=%.2f, H=%.1f)\n", elevatorNewMaxPower, elevatorNewTargetHeightMm);         
        }
        break;
    }
  }
}

// Returns true if the elevator is within tolerance of the target height
bool ElevatorSub::isElevatorAtTarget() {
 if ((fabs(elevatorTargetHeightMm - getElevatorHeight()) < ELEVATOR_POSITION_TOLERANCE_MM) && 
      (fabs(getElevatorVelocity() < ELEVATOR_VELOCITY_TOLERANCE_MM_S))) {
      return true;
 } else {
   return false;
 }
}

// This is the state machine that manages the elevator's motion.  It should be called on at a regular time interval
void ElevatorSub::updateElevatorStateMachine() {
  double newPower = 0.0;
  double currentHeightMm = getElevatorHeight();

  // Apply any new inputs
  if(elevatorNewStateParameters) {
    elevatorState = elevatorNewState;
    elevatorControlMode = elevatorNewControlMode;
    elevatorMaxPower = elevatorNewMaxPower;
    elevatorTargetHeightMm = elevatorNewTargetHeightMm;
    elevatorNewStateParameters = false;
    logger.send(logger.ELEVATOR, "ESM: New (S=%d, M=%d, P=%.2f, H=%.1f)\n", 
        elevatorState, elevatorControlMode, elevatorNewMaxPower, elevatorNewTargetHeightMm);
  }

  // In auto mode
  //  - The target height and max power are set by the caller
  // In manual mode
  // - If the joystick is outside the dead zone, the target height is set to the highest/lowest 
  //   allowable height with the max power based on the joystick value.  
  // - If the max power is close to zero (in the dead zone), the target height is set to the 
  //   current height.
  switch(elevatorState) {
    case ELEVATOR_STATE_IDLE:
      // Don't drive motors, leave as is
      newPower = 0.0;
      break;

    case ELEVATOR_STATE_HOLDING:
      // Give the motor just enough power to keep the current position
      newPower = calcElevatorHoldPower(currentHeightMm, elevatorTargetHeightMm);

      logger.send(logger.ELEVATOR, "ESM: Holding (P=%.2f)\n", newPower);
      break;

    case ELEVATOR_STATE_MOVING:
      if(isElevatorBlocked(currentHeightMm, elevatorTargetHeightMm)) {
        elevatorBlockedHeightMm = currentHeightMm;
        elevatorState = ELEVATOR_STATE_INTERRUPTED;
      }
      else if(isElevatorAtTarget()) {
        elevatorState = ELEVATOR_STATE_HOLDING;
      }
      else {
        newPower = calcElevatorMovePower(currentHeightMm, elevatorTargetHeightMm, elevatorMaxPower);
      }
      logger.send(logger.ELEVATOR, "ESM: Moving (P=%.2f,s=%d,b=%.1f,c=%.1f)\n", newPower, elevatorState, elevatorBlockedHeightMm, currentHeightMm);
      break;

    case ELEVATOR_STATE_INTERRUPTED:
      if(!isElevatorBlocked(currentHeightMm, elevatorTargetHeightMm)) {
        elevatorState = ELEVATOR_STATE_MOVING;
      }
      else {
         newPower = calcElevatorHoldPower(currentHeightMm, elevatorBlockedHeightMm);
      }
      logger.send(logger.ELEVATOR, "ESM: Blocked (P=%.2f,s=%d,b=%.1f,c=%.1f)\n", newPower, elevatorState, elevatorBlockedHeightMm, currentHeightMm);
      break;

    default:
      // This should never happen.  Best thing we can do is hold our current position.
      logger.send(logger.ASSERTS, "Elevator state machine entered invalid state (%d).  Fix the code!\n", elevatorState);     
      elevatorState = ELEVATOR_STATE_HOLDING;
      break;
  }

  if(newPower != elevatorLastPower) {
    setElevatorMotorPower(newPower);
    elevatorLastPower = newPower;
    logger.send(logger.ELEVATOR, "ESM: New power = %f (S=%d, M=%d, P=%.2f, H=%.1f)\n", newPower,
        elevatorState, elevatorControlMode, elevatorNewMaxPower, elevatorNewTargetHeightMm);
  }
}

bool ElevatorSub::isElevatorBlocked(double currentHeightMm, double targetHeightMm) {
  double direction = 1.0;
  
  if(currentHeightMm > targetHeightMm) {
    direction = -1.0;
  }

  // TODO: implement
  // At max height - tolerance (going up)
  if (((currentHeightMm >= ELEVATOR_MAX_HEIGHT_MM) && (direction > 0)) || 
      ((currentHeightMm < ELEVATOR_MIN_HEIGHT_MM) && (direction < 0))) {
    return true;
  }
  // At min height - tolerance (going down)
  // Upper limit switch hit (going up)
  // Lower limit switch hit (going down)
  // Manipulator is not far enough forward
  return false;
}

double ElevatorSub::calcElevatorHoldPower(double currentHeightMm, double targetHeightMm) {
  // TODO:  Determine actual value for this.  
  // Make propertional to target.
  // Take into consideration elevator gear and if carrying hatch/cargo?
  double holdPower = (targetHeightMm -  currentHeightMm) * 0.003;

  return holdPower; 
}

double ElevatorSub::calcElevatorMovePower(double currentHeightMm, double targetHeightMm, double maxElevatorPower) {
  double direction = 1.0;
  double newPower = 0;
  
  if(currentHeightMm > targetHeightMm) {
    direction = -1.0;
  }
  
  // TODO: Use better values
  if(fabs(currentHeightMm - targetHeightMm) > 100) {
    newPower = maxElevatorPower * direction;
  }
  else {
    newPower = std::min(0.2, maxElevatorPower) * direction;
  }
  
  return newPower;
}




