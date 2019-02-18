/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

// Pneumatic Control Module Outputs
constexpr int HATCH_GRIPPER_PCM_ID = 0;
constexpr int CLIMB_GEAR_PCM_ID = 1; // Change
constexpr int EXTEND_FOOT_PCM_ID = 2; //change
constexpr int MANIPULATOR_INTAKE_FOLDER_PCM_ID = 3; 

// CanIDs
constexpr int RIGHT_DRIVE_MOTOR_1_CAN_ID = 1;
constexpr int RIGHT_DRIVE_MOTOR_2_CAN_ID = 2;
constexpr int RIGHT_DRIVE_MOTOR_3_CAN_ID = 3;
constexpr int LEFT_DRIVE_MOTOR_1_CAN_ID = 5;
constexpr int LEFT_DRIVE_MOTOR_2_CAN_ID = 6;
constexpr int LEFT_DRIVE_MOTOR_3_CAN_ID = 7;

constexpr int BALL_INTAKE_WHEELS_MOTOR_CAN_ID = 10;
constexpr int BALL_INTAKE_TOP_FLIP_MOTOR_1_CAN_ID = 11;
constexpr int BALL_INTAKE_BOTTOM_FLIP_MOTOR_2_CAN_ID = 9;

constexpr int ELEVATOR_MOTOR_1_CAN_ID = 4;
constexpr int ELEVATOR_MOTOR_2_CAN_ID = 8;

constexpr int MANIPULATOR_FLIPPER_MOTOR_CAN_ID = 16;
constexpr int MANIPULATOR_LEFT_INTAKE_MOTOR_CAN_ID = 13;
constexpr int MANIPULATOR_RIGHT_INTAKE_MOTOR_CAN_ID = 14;

//DIOs
constexpr int INTAKE_LIMIT_DIO = 9;

constexpr int INTAKE_MOTOR_ENC1_DIO = 2;
constexpr int INTAKE_MOTOR_ENC2_DIO = 3;

