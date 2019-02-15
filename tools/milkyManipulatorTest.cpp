//clang 3.8.0

// Example program
// Example program
#include <iostream>
#include <string>
#include <utility>
#include <cmath>

using namespace std;

    double getCircleCentreX(double targetX, double targetY, double robotX, double robotY, double scoringFaceAngle){
        return ((robotY*robotY) + (2*robotY * targetX * tan(-scoringFaceAngle)) - (2*robotY*targetY) - (2*targetX*targetY*tan(-scoringFaceAngle)) + (robotX*robotX) - (targetX*targetX) + (targetY*targetY))/ (2*((robotY*tan(-scoringFaceAngle)) - (targetY*tan(-scoringFaceAngle)) + robotX - targetX ));
    }

    double getCircleCentreY(double circleCentreX, double scoringFaceAngle, double targetX, double targetY){
        if(tan(fabs(scoringFaceAngle)) > 600000){
        //vertical scoring face
            cout<<"yep"<<endl;
            return ((targetY/2)-((circleCentreX*circleCentreX)/(2*targetY)));
        }
        return (tan(-scoringFaceAngle) * circleCentreX + (targetY - tan(-scoringFaceAngle) * targetX));
    }
    double getRobotTargetAngle(double robotHeading, double cameraAngle, double distance, double scoringFaceAngle, bool comments){
        robotHeading = robotHeading *(M_PI/180);
        cameraAngle = cameraAngle *(M_PI/180);
        scoringFaceAngle = scoringFaceAngle*(M_PI/180);
        double robotX = 0.0;
        double robotY = 0.0;
        double targetX = (distance*cos(((M_PI/2)-robotHeading-cameraAngle)));
        double targetY = (distance*sin(((M_PI/2)-robotHeading-cameraAngle)));
        double circleX = getCircleCentreX(targetX, targetY, robotX, robotY, scoringFaceAngle);
        double circleY = getCircleCentreY(circleX, scoringFaceAngle, targetX, targetY);
        double r = sqrt(pow(robotX-circleX, 2) + pow(robotY - circleY, 2));
        if(comments){
        cout<<"Robot Heading "<<robotHeading<<endl;
        cout<<"Camera Angle "<<cameraAngle<<endl;
        cout<<"Scoring Face Angle "<<scoringFaceAngle<<endl;
        cout<<"cos of pi/2 : "<<cos(M_PI/2)<<endl; 
        cout<<"Target X "<<targetX << endl;
        cout<<"Target Y "<<targetY<< endl;
        cout<<"Circle X "<<circleX<< endl;
        cout<<"Circle Y "<<circleY<< endl;
        cout<<"R "<<r<< endl;
        cout<<"top half: "<< (r*r)+(r*r)- sqrt(pow(robotX-circleX, 2) + pow(robotY-(circleY + r), 2))<< endl;
        cout<<"everything but inverse cos "<<((r*r)+(r*r) - sqrt(pow(robotX-circleX, 2) + pow(robotY-(circleY + r), 2)))/(2*r*r)<<endl;
        cout<<"Final Angle"<<endl;
        }
        return 90+(((acos(((r*r)+(r*r) - sqrt(pow(robotX-circleX, 2) + pow(robotY-(circleY + r), 2)))/(2*r*r))))*(180/M_PI)); //in degrees
        //return 90+(acos((pow(robotX-targetX, 2) + pow(robotY-targetY, 2) - 2*(r*r))/-(2*r*r)))*(180/M_PI);
        //return 90+sqrt()
    }
    
int main(){
    cout<<tan(3*M_PI/2)<<endl;

    getRobotTargetAngle(35.0,0.0,1000.0,151.25, true);
    for(int i = 500; i > 0; i = i-10){
    cout<<getRobotTargetAngle(120.0,0.0,i,90.0, false)<<endl;
    }
}
