/*
 * SilkyMotionManager.h
 *
 *  Created on: Jan 28, 2017
 *      Author: sam_000
 *
 *  Thanks 2702! :)
 */

#ifndef SRC_COMPONENTS_SILKYMOTIONMANAGER_H_
#define SRC_COMPONENTS_SILKYMOTIONMANAGER_H_

#include <vector>
#include <utility>

struct PathInfo {
	double dis;
	double ang;
	double lin_vel;
	double lin_accel;
	double ang_vel;
};

class SilkyMotionManager {
private:
	std::vector<double> maxSpeed;
	std::vector<double> actualSpeed;
	std::vector<double> timestamps;
	std::vector<double> dis;
  	std::vector<bool> negatives;
	std::vector<double> ang;
	double maxLinAccel, maxLinDecel, maxLinVel, maxAngAccel;
	double stoppingDistanceTolerance, stoppingSpeedTolerance, stoppingAngleTolerance;

	double startTime;
	double lastTime;

	double getAngularTime(double angle);
	double getLinearTime(double dis, double ang, double startSpeed, double endSpeed);

	double getMaxSpeed(double dis, double ang, double maxEndSpeed, bool isSwitching);
	double getActualSpeed(double dis, double ang, double startingActualSpeed, double maxEndSpeed);
	double getTimestamp(double dis, double ang, double startingSpeed, double endingSpeed, double prevTime);
public:
	PathInfo getCurrentPathInfo(double t);
	SilkyMotionManager(std::vector<double> dis, std::vector<double> ang,
			double maxLinAccel, double maxLinDecel, double maxLinVel, double maxAngAccel);

	double getTotalTime();

	std::pair<double, double> execute(double currentLeftPos, double currentRightPos); // returns left speed, right speed to set motors
	bool isFinished(double leftPos, double leftVel, double rightPos, double rightVel, double angle);
	void reset();
};

#endif /* SRC_COMPONENTS_SILKYMOTIONMANAGER_H_ */