/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/VisionSub.h"

VisionSub::VisionSub() : Subsystem("ExampleSubsystem") {}

void VisionSub::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void VisionSub::setBumperPipeline(int pipeLine) { 
  switch (pipeLine){
    case DRIVER_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 1);
      break;
    case VISION_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", 0);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", 0);
      break;
    
    default:
      break;
  }
  
}

// Flipping camera orientation
void VisionSub::setManipulatorPipeline(int pipeLine){
  switch (pipeLine)
  {
    case DRIVER_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", 1);
      break;
    case DRIVER_MODE_FLIPPED:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", 2);
      break;
    case VISION_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", 1);
      break;
    case VISION_MODE_FLIPPED:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", 1);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", 2);
      break;
    default:
      break;
  }
}

double VisionSub::getVisionTarget() {
  
  double xAngle = 9001;
  // if over 9000, the vision target is not picking up anything.
  double TargetMarked = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);

  if (TargetMarked > 0.5) {
    xAngle = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
  }

  return xAngle;
}

double VisionSub::getDistanceFromVision() {
  double size=nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("thori",0.0);
  double a=0.295;
  double b=-70.6;
  double c=5234;
  return a*size*size+b*size+c;
}

double VisionSub::getScoringFaceAngle() {
  double RobotAngle = Robot::drivetrainSub.getAngle();
  double TargetAngle[7] = {-151.25, -90, -28.75, 0, 28.75, 90, 151.25};
  double SmallestAngleDifference = 1000;
  int BestTarget;

  for(int i = 0; i < 7; i++){
    double AngleDifference = fabs(RobotAngle - TargetAngle[i]);
    if(AngleDifference <= SmallestAngleDifference){
      SmallestAngleDifference = AngleDifference;
      BestTarget = i;
      
    }
  }
  return TargetAngle[BestTarget];
}

double VisionSub::normalizeAngle(double targetAngle){
  while(targetAngle < -180) {
    targetAngle = targetAngle + 360;
  }
  while(targetAngle > 180) {
    targetAngle = targetAngle - 360;
  }
  return targetAngle;
}