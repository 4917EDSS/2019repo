/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include "subsystems/VisionSub.h"
#include <cmath>
#include <vector>

constexpr int VISION_PIPELINE_NORMAL  = 0;
constexpr int DRIVER_PIPELINE_NORMAL  = 1;
constexpr int DRIVER_PIPELINE_FLIPPED = 2;
constexpr int VISION_PIPELINE_FLIPPED = 3;

constexpr int VISION_CAMMODE  = 0;
constexpr int DRIVER_CAMMODE  = 1;

constexpr int LED_MODE_ON = 3;
constexpr int LED_MODE_OFF = 1;


VisionSub::VisionSub() : Subsystem("ExampleSubsystem") {}

void VisionSub::InitDefaultCommand() {
    setManipulatorPipelineState = DRIVER_MODE_NORMAL;
  // Set the default command for a subsystem here.
  // SetDefaultCommand(new MySpecialCommand());
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
void VisionSub::setBumperPipeline(int pipeLine) { 
  switch (pipeLine){
    case DRIVER_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", LED_MODE_OFF);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", DRIVER_CAMMODE);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", DRIVER_PIPELINE_NORMAL);
      break;
    case VISION_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", LED_MODE_ON);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("camMode", VISION_CAMMODE);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline", VISION_PIPELINE_NORMAL);
      break;
    
    default:
      break;
  }
  
}
double VisionSub::getRobotTargetAngle(double robotHeading, double cameraAngle, double scoringFaceAngle){
        double adjustment;
        std::tie(robotHeading, adjustment) = normalizeAngle(robotHeading);
        robotHeading -= scoringFaceAngle;// Normalizing angles so we can treat all scoring face angles as 0
        double angleToPointDirectlyAtTarget = robotHeading + cameraAngle;
        double targetAngle = 2 * angleToPointDirectlyAtTarget; // This is a geometric proof. See 2019repo/tools/angleproof.png
        double finalUnadjusted = targetAngle + scoringFaceAngle;
        finalUnadjusted = normalizeAngle(finalUnadjusted).first;
        return finalUnadjusted + (360*adjustment); // Bringing us back to reality
}

// Flipping camera orientation
void VisionSub::setManipulatorPipeline(int pipeLine){
  switch (pipeLine)
  {
    case DRIVER_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", LED_MODE_OFF);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", DRIVER_CAMMODE);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", DRIVER_PIPELINE_NORMAL);
      break;
    case DRIVER_MODE_FLIPPED:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", LED_MODE_OFF);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", DRIVER_CAMMODE);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", DRIVER_PIPELINE_FLIPPED);
      break;
    case VISION_MODE_NORMAL:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", LED_MODE_ON);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", VISION_CAMMODE);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", VISION_PIPELINE_NORMAL);
      break;
    case VISION_MODE_FLIPPED:
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("ledMode", LED_MODE_ON);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("camMode", VISION_CAMMODE);
      nt::NetworkTableInstance::GetDefault().GetTable("limelight-two")->PutNumber("pipeline", VISION_PIPELINE_FLIPPED);
      break;
    default:
      break;
  }
  setManipulatorPipelineState = pipeLine;
}


int VisionSub::getManipulatorPipeline() {
  return setManipulatorPipelineState;
}

bool VisionSub::isTargetVisible(int camera){
  double targetMarked = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);
  if (targetMarked > 0){
    return true;
  }
  return false;
}

double VisionSub::getVisionTarget(int camera) {
  return nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
}



double VisionSub::getScoringFaceAngle(int camera) {
  double robotAngle = Robot::drivetrainSub.getAngle();
  robotAngle = normalizeAngle(robotAngle).first;
  if (robotAngle > 180){
    robotAngle -= 360;
  }else if (robotAngle <-180){
    robotAngle += 360;
  }

  std::vector<double> targetAngle = {-151.25, -90, -28.75, 0, 28.75, 90, 151.25};
  double smallestAngleDifference = 1000;
  int bestTarget = 0;
  if (Robot::inBallMode){
    targetAngle = {0.0, 90.0, -90.0};
  }else if (Robot::manipulatorSub.isGripperExpanded()){
    if (robotAngle > 135||robotAngle <-135){
      return 180.0; 
    }
    
  }

  for(int i = 0; i < (int)targetAngle.size(); i++){
    double angleDifference = fabs(robotAngle - targetAngle[i]);
    if(angleDifference > 180){
      angleDifference = fabs(angleDifference-360);
    }
    if(angleDifference <= smallestAngleDifference){
      smallestAngleDifference = angleDifference;
      bestTarget = i;
    }
  }
  return targetAngle[bestTarget];
}

std::pair<double,double> VisionSub::normalizeAngle(double angle){
        double adjustment = 0;
        double result = angle/360;
        if (angle>= 360) {
            adjustment = floor(result);
        } else if (angle <= -360){
            adjustment = ceil(result);
        }
        double fraction = result - adjustment;
        angle = fraction *360;
        
        
        return std::make_pair(angle, adjustment); 
}
    
