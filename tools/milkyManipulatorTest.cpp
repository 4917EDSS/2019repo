#include <iostream>
#include <string>
#include <utility>
#include <cmath>
#include <tuple>


using namespace std;
    
    pair<double,double> normalizeAngle(double angle){
        double adjustment = 0;
        double result = angle/360;
        if (angle>= 360) {
            adjustment = floor(result);
        } else if (angle <= -360){
            adjustment = ceil(result);
        }
        double fraction = result - adjustment;
        angle = fraction *360;
        
        
        return make_pair(angle, adjustment); 
    }
    
    double getRobotTargetAngle(double robotHeading, double cameraAngle, double scoringFaceAngle){
        //cout<<robotHeading<<endl;
        double adjustment;
        std::tie(robotHeading, adjustment) = normalizeAngle(robotHeading);
        robotHeading -= scoringFaceAngle;// Normalizing angles so we can treat all scoring face angles as 0

        //cout<<"Second robot heading "<<robotHeading<<endl;
        double angleToPointDirectlyAtTarget = robotHeading + cameraAngle;
        //cout<<angleToPointDirectlyAtTarget<<endl;
        double targetAngle = 2 * angleToPointDirectlyAtTarget; // This is a geometric proof. See 2019repo/tools/angleproof.png
        //cout<<targetAngle<<endl;
        double finalUnadjusted = targetAngle + scoringFaceAngle;
        finalUnadjusted = normalizeAngle(finalUnadjusted).first;
        //cout<<finalUnadjusted<<endl;
        //cout<<"Adjustment "<<adjustment<<endl;
        return finalUnadjusted + (360*adjustment); // Bringing us back to reality
      
    }
    
int main(){

    cout<<"Expevting 10 "<<getRobotTargetAngle(10.0,-5.0,0.0)<<endl;
    cout<<"Expecting 150 "<<getRobotTargetAngle(120.0,0.0,90.0)<<endl;
    cout<<"Expecting -136 "<<getRobotTargetAngle(-123.0, 10.0,-90.0)<<endl;
    cout<<"Expecting 15 "<<getRobotTargetAngle(0.0,-15.0,-45.0)<<endl;
    cout<<"Expecting 180 "<<getRobotTargetAngle(180.0, 0.0,-180.0)<<endl;
    cout<<"Expecting 730 "<<getRobotTargetAngle(720.0, 5.0, 0.0)<<endl;
    cout<<"Expecting -37 "<<getRobotTargetAngle(1.0, 3.0, 45.0)<<endl;
    cout<<"Expecting -778 "<<getRobotTargetAngle(-724.0, -70.0,-90.0)<<endl;
    cout<<"Expecting -688 "<<getRobotTargetAngle(-634.0, -70.0,0.0)<<endl;
    cout<<"Expecting 950 ish "<<getRobotTargetAngle(925.0, 2.0, 180.0)<<endl;
    cout<<"Expecting -580 ish "<<getRobotTargetAngle(-560.0, 11.0, 180.0)<<endl;
}