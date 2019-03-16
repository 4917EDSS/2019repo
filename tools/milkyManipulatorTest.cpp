#include <iostream>
#include <string>
#include <utility>
#include <cmath>

using namespace std;

    double getRobotTargetAngle(double robotHeading, double cameraAngle, double scoringFaceAngle){
        robotHeading -= scoringFaceAngle; // Normalizing angles so we can treat all scoring face angles as 0
        double angleToPointDirectlyAtTarget = robotHeading + cameraAngle;
        double targetAngle = 2 * angleToPointDirectlyAtTarget; // This is a geometric proof. See 2019repo/tools/angleproof.png
        return targetAngle + scoringFaceAngle; // Bringing us back to reality
    }
    
int main(){

    cout<<getRobotTargetAngle(10.0,-5.0,0.0)<<endl;
    cout<<getRobotTargetAngle(120.0,0.0,90.0)<<endl;
    cout<<getRobotTargetAngle(-123.0, 10.0,-90.0)<<endl;
    cout<<getRobotTargetAngle(0.0,-15.0,-45.0)<<endl;
    cout<<getRobotTargetAngle(180.0, 0.0,-180.0)<<endl;
}
