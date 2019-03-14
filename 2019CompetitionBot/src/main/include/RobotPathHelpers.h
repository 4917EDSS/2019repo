#include <utility>
#define _USE_MATH_DEFINES
#include <cmath>

inline double getCircleCentreX(double targetX, double targetY, double robotX, double robotY, double scoringFaceAngle){
    return ((robotY*robotY) + (2*robotY * targetX * tan(-scoringFaceAngle)) - (2*robotY*targetY) - (2*targetX*targetY*tan(-scoringFaceAngle)) + (robotX*robotX) - (targetX*targetX) + (targetY*targetY))
    / (2*((robotY*tan(-scoringFaceAngle)) - (targetY*tan(-scoringFaceAngle)) + robotX - targetX ));
}

inline double getCircleCentreY(double circleCentreX, double scoringFaceAngle, double targetX, double targetY){
    return (tan(-scoringFaceAngle) * circleCentreX + (targetY - tan(-scoringFaceAngle) * targetX));
}

inline double GetRobotTargetAngle(double robotHeading, double cameraAngle, double distance, double scoringFaceAngle){
        robotHeading = robotHeading *(M_PI/180);
        cameraAngle = cameraAngle *(M_PI/180);
        scoringFaceAngle = scoringFaceAngle*(M_PI/180);
        //cout<<"Robot Heading "<<robotHeading<<endl;
        //cout<<"Camera Angle "<<cameraAngle<<endl;
        //cout<<"Scoring Face Angle "<<scoringFaceAngle<<endl;
        //cout<<"cos of pi/2 : "<<cos(M_PI/2)<<endl; 
        double robotX = 0.0;
        double robotY = 0.0;
        double targetX = (distance*cos((M_PI/2)-robotHeading-cameraAngle));
        //cout<<"Target X "<<targetX << endl;
        double targetY = (distance*sin((M_PI/2)-robotHeading-cameraAngle));
        //cout<<"Target Y "<<targetY<< endl;
        double circleX = getCircleCentreX(targetX, targetY, robotX, robotY, scoringFaceAngle);
        //cout<<"Circle X "<<circleX<< endl;
        double circleY = getCircleCentreY(circleX, scoringFaceAngle, targetX, targetY);
        //cout<<"Circle Y "<<circleY<< endl;
        double r = sqrt(pow(robotX-circleX, 2) + pow(robotY - circleY, 2));
        //cout<<"R "<<r<< endl;
        //cout<<"top half"<< (r*r)+(r*r)- sqrt(pow(robotX-circleX, 2) + pow(robotY-(circleY + r), 2))<< endl;
        //cout<<"everything but inverse cos"<<((r*r)+(r*r) - sqrt(pow(robotX-circleX, 2) + pow(robotY-(circleY + r), 2)))/(2*r*r)<<endl;
        
        //cout<<"Final Angle"<<endl;
        
        return (acos(((r*r)+(r*r) - sqrt(pow(robotX-circleX, 2) + pow(robotY-(circleY + r), 2)))/(2*r*r)));
    }
