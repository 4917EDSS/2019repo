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
#include "networktables/NetworkTableInstance.h"

DrivetrainSub Robot::drivetrainSub;
BallIntakeSub Robot::ballIntakeSub;
ElevatorSub Robot::elevatorSub;
ManipulatorSub Robot::manipulatorSub;
ClimbSub Robot::climbSub;
VisionSub Robot::visionSub;
OI Robot::oi;

bool Robot::inBallMode;
bool Robot::stateMachinesReset;


void Robot::RobotInit() {
  // No Auto command this year
  // frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  // Setup logging system
  std::string syslogTargetAddress = (Preferences::GetInstance())->GetString("SyslogTargetAddress", "10.49.17.30");

  // These should stay on during competition
  logger.enableChannels(logger.WARNINGS | logger.ERRORS | logger.ASSERTS);

  // Can use these for debugging.  Comment/uncomment as needed
  logger.enableChannels(logger.DEBUGGING);
  //logger.enableChannels(logger.DRIVETRAIN);
  //logger.enableChannels(logger.VISION);
  //logger.enableChannels(logger.PERIODIC);
  logger.enableChannels(logger.CMD_TRACE);
  //logger.enableChannels(logger.ELEVATOR);
  //logger.enableChannels(logger.BALLINTAKE);
  //logger.enableChannels(logger.MANIPULATOR);
  //logger.enableChannels(logger.WITH_JOYSTICK_TRACE);
  logger.enableChannels(logger.CLIMB);

  logger.addOutputPath(new frc4917::ConsoleOutput());						            // Enable console output and/or
  logger.addOutputPath(new frc4917::SyslogOutput(syslogTargetAddress));		  // Enable syslog output
  logger.send(logger.DEBUGGING, "Robot code started @ %f\n", GetTime());

  std::cout << "Starting version 1.7\n";

  Robot::inBallMode = true;
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
  Robot::visionSub.setBumperPipeline(false);
  Robot::ballIntakeSub.foldIntakeArms();
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_AUTO, 0.5, INTAKE_NEUTRAL_ANGLE);

  if(!stateMachinesReset) {
    resetStateMachines();
    stateMachinesReset = true;
  }

  // No auto command this year
  /*
  m_autonomousCommand = m_chooser.GetSelected();
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Start();
  }
  */

  // But probably need to deal with initial game piece
  // Hatch:
  //  - inBallMode = false
  //  - intake hatch (gripper expand, roll wheels back a bit): ExpandHatchGripperGrp
  // Cargo:
  //  - inBallMode = true
  // None:
  //  - do nothing
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
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }

  if(!stateMachinesReset) {
    resetStateMachines();
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
  // Robot::climbSub.updateShuffleBoard();
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
  double targetPerecentArea = table->GetNumber("ta", 0.0);
  double targetSkew = table->GetNumber("ts", 0.0);
  double targetHorizontalOffset = table->GetNumber("tx", 0.0);
  double targetVerticalOffset = table -> GetNumber("ty", 0.0);
  bool foundTarget = table->GetNumber("tv", 0.0);
  double limelightLatency = table->GetNumber("tl", 0.0);
  double targetShort = table->GetNumber("tshort", 0.0);
  double targetLong =table->GetNumber("tlong", 0.0);
  double targetHorizontalLength = table->GetNumber("thoriz", 0.0);
  double targetVerticalLength =  table ->GetNumber("tvert", 0.0);

  frc::SmartDashboard::PutNumber("Target Percent Area", targetPerecentArea);
  frc::SmartDashboard::PutNumber("Target Skew", targetSkew);
  frc::SmartDashboard::PutNumber("Target Horizontal Offset" ,targetHorizontalOffset);
  frc::SmartDashboard::PutNumber("Target Vertical Offset" ,targetVerticalOffset);
  frc::SmartDashboard::PutNumber("Target is in view", foundTarget);
  frc::SmartDashboard::PutNumber("Limelight Latency", limelightLatency);
  frc::SmartDashboard::PutNumber("Target's shortest side of fitted bounding box", targetShort);
  frc::SmartDashboard::PutNumber("Target's longest side of fitted bounding box", targetLong);
  frc::SmartDashboard::PutNumber("Target's horiz. sidelength of rough bounding box", targetHorizontalLength);
  frc::SmartDashboard::PutNumber("Target's vert. sidelength of rough bounding box", targetVerticalLength);
  
  frc::SmartDashboard::PutNumber("Elevator Height", elevatorSub.getElevatorHeight());
  frc::SmartDashboard::PutNumber("Manip Angle", manipulatorSub.getFlipperAngle());
  frc::SmartDashboard::PutNumber("Manip Wheels", manipulatorSub.getIntakePower());
  frc::SmartDashboard::PutBoolean("Gripper Exp.", manipulatorSub.isGripperExpanded());
  frc::SmartDashboard::PutBoolean("Ball-In Sensor", manipulatorSub.isBallIn());
  frc::SmartDashboard::PutNumber("Intake Arm Angle",ballIntakeSub.getIntakeArmAngle());
  frc::SmartDashboard::PutBoolean("Intake Unfolded",ballIntakeSub.isIntakeUnfolded());
  frc::SmartDashboard::PutNumber("Yaw Angle", drivetrainSub.getAngle());  
}

void Robot::resetStateMachines() {
  Robot::elevatorSub.setElevatorHeight(ELEVATOR_MODE_DISABLED, 0, 0);
  Robot::manipulatorSub.setFlipperAngle(FLIPPER_MODE_DISABLED, 0, 0);
  Robot::ballIntakeSub.setIntakeArmAngle(INTAKE_ARM_MODE_DISABLED, 0, 0);
}
