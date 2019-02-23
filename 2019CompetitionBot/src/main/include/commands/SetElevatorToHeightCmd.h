/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "commands/frc4917Cmd.h"

constexpr double UP_CLIMB_POSITION = 60.5; //change
  constexpr double DOWN_CLIMB_POSITION = -30.0; //change, check if zero is the middle position (at rest)
  constexpr double LOW_HATCH_HEIGHT = 20.0; //change, low spot on cargo ship, hatch intake
  constexpr double MIDDLE_HATCH_HEIGHT= 40.0;//change, middle spot on rocket
  constexpr double UP_HATCH_HEIGHT = 60.0; //change, top spot on rocket.
  constexpr double BALL_PICKUP = 23.0; //change, ball pickup
  constexpr double LOW_BALL_DROPOFF = 25.0; //change, ball dropoff on cargo ship
  constexpr double MIDDLE_BALL_DROPOFF = 45.0; //change, ball dropoff on middle rocket slot
  constexpr double UP_BALL_DROPOFF = 65.0; //change, ball dropoff on upper rocket slot
  constexpr double FLOOR_PICKUP = 5.0; //change, ball pickup off floor 

class SetElevatorToHeightCmd : public frc4917Cmd
{

public:


  SetElevatorToHeightCmd(double height);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;

private:
  double height;
};
