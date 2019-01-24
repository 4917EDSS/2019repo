#include <utility>
#include <cmath>

double getCircleCenterX(double targetX, double targetY, double robotX, double robotY, double ScoringFaceAngle){
    return ((robotY*robotY) + (2*robotY * targetX * tan(-ScoringFaceAngle)) - (2*robotY*targetY) - (2*targetX*targetY*tan(-ScoringFaceAngle)) + (robotX*robotX) - (targetX*targetX) + (targetY*targetY))
    / (2*((robotY*tan(-ScoringFaceAngle)) - (targetY*tan(-ScoringFaceAngle)) + robotX - targetX ));
}

double getCircleCenterY(double CircleCenterX, double ScoringFaceAngle, double targetX, double targetY){
    return (tan(-ScoringFaceAngle) * CircleCenterX + (targetY - tan(-ScoringFaceAngle) * targetX));
}

double GetRobotTargetAngle(double RobotHeading, double CameraAngle, double Distance, double ScoringFaceAngle, double robotX, double robotY, double targetX, double targetY){
    double circleX = getCircleCenterX(targetX, targetY, robotX, robotY, ScoringFaceAngle);
    double circleY = getCircleCenterY(circleX, ScoringFaceAngle, targetX, targetY);
    double r = sqrt(pow(robotX-circleX, 2) + pow(robotY - circleY, 2));
    return (acos((r*r)+(r*r) - sqrt(pow(robotX-circleX, 2) + pow(robotY-(circleY + r), 2)))/(2*r*r))  + 90;
}
