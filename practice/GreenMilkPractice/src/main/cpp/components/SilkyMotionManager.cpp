/*
 * SilkyMotionManager.cpp
 *
 * We assume linear can be carried forward into next step, but always assume there is no angular velocity going into the next one.
 *
 *  Created on: Jan 28, 2017
 *      Author: sam_000
 */

#include <cmath>
#include <iostream>
#include <algorithm>

#include "Components/SilkyMotionManager.h"

PathInfo SilkyMotionManager::getCurrentPathInfo(double currentTime) {
	PathInfo pi; // dis, ang, lin_vel, lin_accel, ang_vel
  bool isNegative = false;
  double time = 0;
	double totalTime = 0;
	double dist = 0;
	double angle = 0;
	double startSpeed = 0;
	double endSpeed = 0;
	double accmulatedAng = 0;
	double accmulatedDist = 0;
	for (unsigned int i = 1; i < timestamps.size(); i++) {
		if ((currentTime >= timestamps[i - 1]) && (currentTime < timestamps[i])) {
			time = currentTime - timestamps[i - 1];
			totalTime = timestamps[i] - timestamps[i - 1];
			angle = ang[i - 1];
			dist = dis[i - 1];
			startSpeed = actualSpeed[i - 1];
			endSpeed = actualSpeed[i];
      isNegative = negatives[i-1];
			break;
		} else {
			accmulatedAng += ang[i - 1];
      if(negatives[i-1]){
        accmulatedDist -= dis[i - 1];
      }else{
		  	accmulatedDist += dis[i - 1];
      }
		}
	}
	if (currentTime > timestamps[timestamps.size() - 1]){
		pi.ang_vel = 0;
		pi.ang = accmulatedAng;
		pi.lin_accel = 0;
		pi.lin_vel = 0;
		pi.dis = accmulatedDist;
		return pi;
	}

	pi.ang_vel = angle / totalTime;
	pi.ang = pi.ang_vel * time;

	double radius = dist/(fabs(angle)*M_PI/180.0); // Converting to radians to get radius of curvature
	double angularMax = sqrt(maxAngAccel * radius);
  if ((angle == 0) || (angularMax > maxLinVel)) {
    angularMax = maxLinVel;
  }
	double adjustedMaxVel = angularMax;

	double disToMaxVel = 0.5 * (pow(adjustedMaxVel,2) - pow(startSpeed , 2)) / maxLinAccel;
	double disToEndVel = 0.5 * (pow(adjustedMaxVel,2) - pow(endSpeed, 2)) / maxLinDecel;
	if (disToMaxVel + disToEndVel > dist) {
		// Don't get to full speed, so triangle instead of trapezoid
		// d_accel = (v^2-startSpeed^2)/(2*a_accel)
		// d_decel = (v^2-endSpeed^2)/(2*a_decel)
		// d_total = (v^2-startSpeed^2)/(2*a_accel) + (v^2-endSpeed^2)/(2*a_decel)
		// Rearranging for v gives
		adjustedMaxVel = sqrt((2 * maxLinAccel * maxLinDecel * dist + (maxLinDecel * pow(startSpeed, 2)) + (maxLinAccel * pow(endSpeed, 2))) / (maxLinDecel + maxLinAccel));
		disToMaxVel = 0.5 * (pow(adjustedMaxVel,2) - pow(startSpeed, 2)) / maxLinAccel;
		disToEndVel = 0.5 * (pow(adjustedMaxVel,2) - pow(endSpeed, 2)) / maxLinDecel;
	}
	double timeToMaxVel = (adjustedMaxVel - startSpeed) / maxLinAccel;
	double timeAtMaxVel = (dist - disToMaxVel - disToEndVel) / adjustedMaxVel;

	if (time < timeToMaxVel) {
		// We are currently accelerating
		pi.lin_accel = maxLinAccel;
		pi.lin_vel = startSpeed + maxLinAccel * time;
		pi.dis = (startSpeed * time) + (0.5 * maxLinAccel * pow(time, 2));
	} else if (time < (timeToMaxVel + timeAtMaxVel)) {
		pi.lin_accel = 0;
		pi.lin_vel = adjustedMaxVel;
		pi.dis = disToMaxVel + adjustedMaxVel * (time - timeToMaxVel);
	} else {
		pi.lin_accel = -maxLinDecel;
		double decelTime = time - timeAtMaxVel - timeToMaxVel;
		pi.lin_vel = adjustedMaxVel - (maxLinDecel * decelTime);
		pi.dis = disToMaxVel + (timeAtMaxVel * adjustedMaxVel) + (adjustedMaxVel * decelTime) - (0.5 * maxLinDecel * pow(decelTime, 2));
	}

  if(isNegative){
    pi.lin_accel = -pi.lin_accel;
    pi.lin_vel = -pi.lin_vel;
    pi.dis = -pi.dis;
  }

	pi.dis += accmulatedDist;
	pi.ang += accmulatedAng;

	return pi;
}

double SilkyMotionManager::getAngularTime(double angle) {
	//have to do hard math
	//assuming we can ignore acceleration, deceleration, and angular component
	return 0;//(fabs(angle) / maxAngVel);
}

double SilkyMotionManager::getLinearTime(double dis, double ang, double startSpeed, double endSpeed) {
	//Assuming no crazy curves in spline, and giving reasonable headroom for acceleration + speed. We will tell program "max" is actually lower than max, since we are only programming for the left side. If the right side needs to turn faster than the right, the combination of a smaller max and a reasonable curvature will allow it to get ahead.
	//This means the left side will go "as fast as possible" in this model.
	double radius = dis/(fabs(ang)*M_PI/180.0); // Converting to radians to get radius of curvature
	double angularMax = sqrt(maxAngAccel * radius);
  if ((ang == 0) || (angularMax > maxLinVel))  {
    angularMax = maxLinVel;
  }
  double adjustedMaxVel = angularMax;

	double disToMaxVel = 0.5 * (pow(adjustedMaxVel,2) - pow(startSpeed , 2)) / maxLinAccel;
	double disToEndVel = 0.5 * (pow(adjustedMaxVel,2) - pow(endSpeed, 2)) / maxLinDecel;
	if (disToMaxVel + disToEndVel > dis) {
		// Don't get to full speed, so triangle instead of trapezoid
		// d_accel = (v^2-startSpeed^2)/(2*a_accel)
		// d_decel = (v^2-endSpeed^2)/(2*a_decel)
		// d_total = (v^2-startSpeed^2)/(2*a_accel) + (v^2-endSpeed^2)/(2*a_decel)
		// Rearranging for v gives
		adjustedMaxVel = sqrt((2 * maxLinAccel * maxLinDecel * dis + (maxLinDecel * pow(startSpeed, 2)) + (maxLinAccel * pow(endSpeed, 2))) / (maxLinDecel + maxLinAccel));
		disToMaxVel = 0.5 * (pow(adjustedMaxVel,2) - pow(startSpeed, 2)) / maxLinAccel;
		disToEndVel = 0.5 * (pow(adjustedMaxVel,2) - pow(endSpeed, 2)) / maxLinDecel;
	}
	double timeToMaxVel = (adjustedMaxVel - startSpeed) / maxLinAccel;
	double timeAtMaxVel = (dis - disToMaxVel - disToEndVel) / adjustedMaxVel;
	double timeToEnd = (adjustedMaxVel - endSpeed) / maxLinDecel;
	return timeToEnd + timeToMaxVel + timeAtMaxVel;
}

// Getting the absolute fastest we can start the current motion, given the absolute fastest we can be going at the end of the motion.
// For now, using simplifying assumptions that it is just the lowest of (A) linear decel required and (B) max centripetal force
double SilkyMotionManager::getMaxSpeed(double dis, double ang, double maxEndSpeed, bool isSwitching) {
  if (isSwitching) {
    return 0;
  }

	double radius = dis/(fabs(ang)*M_PI/180.0); // Converting to radians to get radius of curvature
	double angularMax = sqrt(maxAngAccel * radius);
  if (ang == 0) {
    angularMax = maxLinVel;
  }

	// vf^2 = vi^2 + 2*a*d
	double linearMax = sqrt(pow(maxEndSpeed,2) + 2*maxLinDecel*dis);
	return std::min({angularMax, linearMax, maxLinVel});
}

// How fast can we be going when we end the current motion, given how fast we started the motion and how fast we are allowed to end it.
// For now, using simplifying assumption that it is just based on linear acceleration, ignoring angular stuff.
double SilkyMotionManager::getActualSpeed(double dis, double ang, double startingActualSpeed, double maxEndSpeed) {
	// vf^2 = vi^2 + 2*a*d
	double maxAchievableSpeed = sqrt(pow(startingActualSpeed,2) + 2*maxLinAccel*dis);

	double radius = dis/(fabs(ang)*M_PI/180.0); // Converting to radians to get radius of curvature
	double angularMax = sqrt(maxAngAccel * radius);
  if (ang == 0) {
    angularMax = maxLinVel;
  }

	return std::min({maxAchievableSpeed, maxEndSpeed, angularMax});
}

double SilkyMotionManager::getTimestamp(double dis, double ang, double startingSpeed, double endingSpeed, double prevTime) {
  // Ignoring angular time since it's basically impossible to restrain linear speed to non linear time
	return prevTime + getLinearTime(dis, ang, startingSpeed, endingSpeed);
}

std::pair<double, double> SilkyMotionManager::execute(double currentLeftPos, double currentRightPos){
	return std::make_pair(1.0, 1.0);
}

bool SilkyMotionManager::isFinished(double leftPos, double leftVel, double rightPos, double rightVel, double angle){
	return false;
}

void SilkyMotionManager::reset(){

}

double SilkyMotionManager::getTotalTime() {
  return timestamps[timestamps.size() - 1];
}

SilkyMotionManager::SilkyMotionManager(std::vector<double> d, std::vector<double> a,
			double maxLinAccel, double maxLinDecel, double maxLinVel, double maxAngAccel) :
			dis(d), ang(a), maxLinAccel(maxLinAccel), maxLinDecel(maxLinDecel), maxLinVel(maxLinVel), maxAngAccel(maxAngAccel),
			startTime(-1),
			lastTime(0) {
		maxSpeed.resize(dis.size() + 1);
		negatives.resize(dis.size());
		maxSpeed[maxSpeed.size() - 1] = 0;

		for (unsigned int i = 0; i<dis.size(); i++){
			if(dis[i] < 0){
				negatives[i]=true;
				dis[i] = -dis[i];
			} else{
				negatives[i]=false;
			}
		}

		for(int i = dis.size()-1; i>=0; i--){
			if (i==0) {
				maxSpeed[i] = getMaxSpeed(dis[i], ang[i], maxSpeed[i+1], false);
			} else {
				maxSpeed[i] = getMaxSpeed(dis[i], ang[i], maxSpeed[i+1], negatives[i] != negatives[i-1]);
			}
		}
		actualSpeed.resize(dis.size() + 1);
		actualSpeed[0] = 0;
		timestamps.resize(dis.size() + 1);
		timestamps[0] = 0;
		for(unsigned int i = 1; i <= dis.size(); i++){
			actualSpeed[i] = getActualSpeed(dis[i-1], ang[i-1], actualSpeed[i-1], maxSpeed[i]);
			timestamps[i] = getTimestamp(dis[i-1], ang[i-1], actualSpeed[i-1], actualSpeed[i], timestamps[i-1]);
		}
}