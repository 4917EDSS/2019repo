
#pragma once


#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>

struct SparkShuffleboardEntrySet {
  nt::NetworkTableEntry setPower;
  nt::NetworkTableEntry outputCurrent;
  nt::NetworkTableEntry encoderPosition;
  nt::NetworkTableEntry encoderVelocity;
  nt::NetworkTableEntry motorTemperature;
};

