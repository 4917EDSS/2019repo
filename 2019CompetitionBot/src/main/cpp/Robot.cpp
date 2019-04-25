/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "Robot.h"
#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/DrivetrainSub.h"
#include "networktables/NetworkTableInstance.h"
#include "commands/ExpandHatchGripperGrp.h"
#include "commands/SetManipulatorAngleCmd.h"
#include "commands/AutoCenterCargoShipLeftHatchGrp.h"
#include "commands/AutoLeftRocketCloseHatchGrp.h"
#include "commands/AutoRightRocketCloseHatchGrp.h"
#include "commands/SetBallModeCmd.h"
#include "commands/AutoSecondLevelLeftRocketHatchGrp.h"
#include "commands/AutoSecondLevelRightRocketHatchGrp.h"
#include "commands/AutoSecondLevelLeftCargoHatchGrp.h"
#include "commands/AutoSecondLevelRightCargoHatchGrp.h"
#include "commands/AutoVision2ndLevelLCargoHatchGrp.h"
#include "commands/SilkyMotionCmd.h"
#include "commands/AutoVisionFirstLevelLeftRocketTwoHatchGrp.h"
#include "commands/AutoSecondLevelLeftCargoAndRocketHatchGrp.h"
#include "commands/AutoSecondLevelRightCargoAndRocketHatchGrp.h"
#include "commands/AutoVisionFirstLevelRightRocketTwoHatchGrp.h"



DrivetrainSub Robot::drivetrainSub;
BallIntakeSub Robot::ballIntakeSub;
ElevatorSub Robot::elevatorSub;
ManipulatorSub Robot::manipulatorSub;
ClimbSub Robot::climbSub;
VisionSub Robot::visionSub;
OI Robot::oi;

bool Robot::inBallMode;
bool Robot::inClimbMode;
bool Robot::startForwards;
bool Robot::stateMachinesReset;


void Robot::RobotInit() {
  autoChooser.reset(new frc::SendableChooser<std::shared_ptr<frc::Command>>());
  autoChooser->AddOption("Ball", std::shared_ptr<frc::Command>(new SetBallModeCmd()));
  autoChooser->AddOption("Hatch do nothing", std::shared_ptr<frc::Command>(new ExpandHatchGripperGrp()));
  autoChooser->AddOption("Centre Hatch", std::shared_ptr<frc::Command>(new AutoCenterCargoShipLeftHatchGrp()));
  autoChooser->AddOption("Left Hatch", std::shared_ptr<frc::Command>(new AutoLeftRocketCloseHatchGrp()));
  autoChooser->AddOption("Right Hatch", std::shared_ptr<frc::Command>(new AutoRightRocketCloseHatchGrp()));
  autoChooser->AddOption("2 lvl L Hatch", std::shared_ptr<frc::Command>(new AutoSecondLevelLeftRocketHatchGrp()));
  autoChooser->AddOption("2 lvl R Hatch", std::shared_ptr<frc::Command>(new AutoSecondLevelRightRocketHatchGrp()));
  autoChooser->AddOption("1 lvl L Rocket 2 Hatch", std::shared_ptr<frc::Command>(new AutoVisionFirstLevelLeftRocketTwoHatchGrp()));
  autoChooser->AddOption("1 lvl R Rocket 2 Hatch", std::shared_ptr<frc::Command>(new AutoVisionFirstLevelRightRocketTwoHatchGrp()));
  autoChooser->AddOption("2 L Cargo Rocket Hatch", std::shared_ptr<frc::Command>(new AutoSecondLevelLeftCargoAndRocketHatchGrp()));
  autoChooser->AddOption("2 R Cargo Rocket Hatch", std::shared_ptr<frc::Command>(new AutoSecondLevelRightCargoAndRocketHatchGrp()));

	SmartDashboard::PutData("1", new SilkyMotionCmd(std::vector<double> {500, 1700, 400}, std::vector<double> {0, -80, 52}));
  SmartDashboard::PutData("2", new SilkyMotionCmd(std::vector<double> {-400, -2000}, std::vector<double> {30, -2}));
  SmartDashboard::PutData("3", new SilkyMotionCmd(std::vector<double> {2000, 2500, -750, -1200, -750, -750}, std::vector<double> {0, 90, 75, 0, -5, 48}));

  frc::SmartDashboard::PutData("Auto Modes", autoChooser.get());

  // Setup logging system
  std::string syslogTargetAddress = (Preferences::GetInstance())->GetString("SyslogTargetAddress", "10.49.17.30");

  // These should stay on during competition
  logger.enableChannels(logger.WARNINGS | logger.ERRORS | logger.ASSERTS);

  // Can use these for debugging.  Comment/uncomment as needed
  logger.enableChannels(logger.DEBUGGING);
  //logger.enableChannels(logger.DRIVETRAIN);
  //logger.enableChannels(logger.VISION);
  //logger.enableChannels(logger.PERIODIC);
  //logger.enableChannels(logger.CMD_TRACE);
  //logger.enableChannels(logger.ELEVATOR);
  //logger.enableChannels(logger.BALLINTAKE);
  //logger.enableChannels(logger.MANIPULATOR);
  //logger.enableChannels(logger.WITH_JOYSTICK_TRACE);
  //logger.enableChannels(logger.CLIMB);

  logger.addOutputPath(new frc4917::ConsoleOutput());						            // Enable console output and/or
  //logger.addOutputPath(new frc4917::SyslogOutput(syslogTargetAddress));		  // Enable syslog output
  logger.send(logger.DEBUGGING, "Robot code started @ %f\n", GetTime());

  std::cout << "Starting version 2.0\n";

  Robot::inBallMode = true;
  Robot::inClimbMode = false;
  Robot::stateMachinesReset = false;
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 * This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {

}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  stateMachinesReset = false;
}

void Robot::DisabledPeriodic() { 
  frc::Scheduler::GetInstance()->Run(); 
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
  Robot::drivetrainSub.resetAHRS();
  Robot::visionSub.setBumperPipeline(DRIVER_MODE_NORMAL);
  Robot::visionSub.setManipulatorPipeline(DRIVER_MODE_NORMAL);
  Robot::ballIntakeSub.foldIntakeArms();
  Robot::elevatorSub.SetElevatorEncoderZero();
  Robot::ballIntakeSub.SetBallIntakeEncoderZero();
  Robot::manipulatorSub.SetManipulatorEncoderZero();
  Robot::climbSub.SetClimbEncoderZero();
  Robot::drivetrainSub.SetDrivetrainEncoderZero();

  if(!stateMachinesReset) {
    resetStateMachines();
    stateMachinesReset = true;
  }
  
  autoCommand = autoChooser->GetSelected().lock();
  
  autoCommand->Start();

    // if (startForwards){
    //   (new SetManipulatorAngleCmd(90))->Start();
    // }else{
    //   (new SetManipulatorAngleCmd(-90))->Start();
    // }
}

void Robot::AutonomousPeriodic() { 
  //frc::Scheduler::GetInstance()->Run(); 
  
  // This year, Auto runs like Teleop
  TeleopPeriodic();

}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.

  if(!stateMachinesReset) {
    resetStateMachines();
    Robot::visionSub.setBumperPipeline(DRIVER_MODE_NORMAL);
    Robot::visionSub.setManipulatorPipeline(DRIVER_MODE_NORMAL);
    stateMachinesReset = true;
  }
}

void Robot::TeleopPeriodic() { 
  frc::Scheduler::GetInstance()->Run(); 
  
  Robot::elevatorSub.updateElevatorStateMachine();
  Robot::manipulatorSub.updateFlipperStateMachine();
  Robot::ballIntakeSub.updateIntakeArmStateMachine();

  // Enable these as needed since running them all takes too long
  // Robot::drivetrainSub.updateShuffleBoard();
  // Robot::elevatorSub.updateShuffleBoard();
  // Robot::manipulatorSub.updateShuffleBoard();
  // Robot::ballIntakeSub.updateShuffleBoard();
  Robot::climbSub.updateShuffleBoard();
  UpdateSmartDashboard();
}

void Robot::TestPeriodic() {

}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

void Robot::UpdateSmartDashboard() {
   //Limelight Data
  std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  // double targetPerecentArea = table->GetNumber("ta", 0.0);
  // double targetSkew = table->GetNumber("ts", 0.0);
  // double targetHorizontalOffset = table->GetNumber("tx", 0.0);
  // double targetVerticalOffset = table -> GetNumber("ty", 0.0);
  bool foundTarget = table->GetNumber("tv", 0.0);
  // double limelightLatency = table->GetNumber("tl", 0.0);
  // double targetShort = table->GetNumber("tshort", 0.0);
  // double targetLong =table->GetNumber("tlong", 0.0);
  // double targetHorizontalLength = table->GetNumber("thoriz", 0.0);
  // double targetVerticalLength =  table ->GetNumber("tvert", 0.0);

  // frc::SmartDashboard::PutNumber("Target Percent Area", targetPerecentArea);
  // frc::SmartDashboard::PutNumber("Target Skew", targetSkew);
  // frc::SmartDashboard::PutNumber("Target Horizontal Offset" ,targetHorizontalOffset);
  // frc::SmartDashboard::PutNumber("Target Vertical Offset" ,targetVerticalOffset);
  frc::SmartDashboard::PutNumber("Target is in view", foundTarget);
  // frc::SmartDashboard::PutNumber("Limelight Latency", limelightLatency);
  // frc::SmartDashboard::PutNumber("Target's shortest side of fitted bounding box", targetShort);
  // frc::SmartDashboard::PutNumber("Target's longest side of fitted bounding box", targetLong);
  // frc::SmartDashboard::PutNumber("Target's horiz. sidelength of rough bounding box", targetHorizontalLength);
  // frc::SmartDashboard::PutNumber("Target's vert. sidelength of rough bounding box", targetVerticalLength);
  
  frc::SmartDashboard::PutNumber("Elevator Height", elevatorSub.getElevatorHeight());
  frc::SmartDashboard::PutNumber("Manip Angle", manipulatorSub.getFlipperAngle());
  frc::SmartDashboard::PutNumber("Manip Wheels", manipulatorSub.getIntakePower());
  frc::SmartDashboard::PutBoolean("Gripper Exp.", manipulatorSub.isGripperExpanded());
  frc::SmartDashboard::PutBoolean("Ball-In Sensor", manipulatorSub.isBallIn());
  frc::SmartDashboard::PutNumber("Intake Arm Angle",ballIntakeSub.getIntakeArmAngle());
  frc::SmartDashboard::PutBoolean("Intake Unfolded",ballIntakeSub.isIntakeUnfolded());
  frc::SmartDashboard::PutNumber("Yaw Angle", drivetrainSub.getAngle());
  frc::SmartDashboard::PutNumber("Drivetrain Velocity", drivetrainSub.getVelocity());
  frc::SmartDashboard::PutNumber("Drivetrain Left Encoder", drivetrainSub.getLeftEncoder());
  frc::SmartDashboard::PutNumber("Drivetrain Right Encoder", drivetrainSub.getRightEncoder());
  frc::SmartDashboard::PutNumber("Climb Height", climbSub.getClimbPosition());
  frc::SmartDashboard::PutNumber("Pitch", drivetrainSub.getPitchAngle());
  frc::SmartDashboard::PutNumber("Speedometer", drivetrainSub.getVelocity());

}

void Robot::resetStateMachines() {
  Robot::elevatorSub.setElevatorHeight(ELEVATOR_MODE_DISABLED, 0, 0);
  Robot::manipulatorSub.setFlipperAngle(FLIPPER_MODE_DISABLED, 0, 0);
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_DISABLED, 0, 0);
}
