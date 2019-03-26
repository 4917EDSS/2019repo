#ifndef SilkyMotionCmd_H
#define SilkyMotionCmd_H
#include <vector>
#include  "Components/SilkyMotionManager.h"
#include "../Robot.h"


const double MAX_LIN_ACCEL=2200.0;
const double MAX_LIN_DECEL=3000.0;
const double MAX_LIN_VEL=2400.0;
const double MAX_ANG_ACCEL=800.0;

const double P_DIS=0.0075;
const double D_DIS=0.000002;
const double A_DIS=0.3*(0.8/2650.0); //2650 IS OLD MAX_LIN_VEL
const double V_DIS=1.0*(0.8/2650.0);
const double P_ANG=0.039;
//When we drove at these speeds we were at 0.3 power we got 50 degrees per second
const double V_ANG=0.8*(0.3/50.0);

constexpr float DRIVE_DISTANCE_TOLERANCE = 15.0;
constexpr float DISTANCE_SPEED_TOLERANCE = 10.0;
constexpr float DRIVE_TURN_TOLERANCE = 2.0;
constexpr float DRIVE_RATE_TOLERANCE = 1;

class SilkyMotionCmd : public frc::Command {
public:
	SilkyMotionCmd(std::vector<double> dis, std::vector<double> ang);
	SilkyMotionCmd(std::vector<double> dis, std::vector<double> ang, double decel);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	SilkyMotionManager smm;
	double prevError;
	double prevTime;
};

#endif  // SilkyMotionCmd_H