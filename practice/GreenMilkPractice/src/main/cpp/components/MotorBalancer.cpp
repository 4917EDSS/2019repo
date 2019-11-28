/*
 * MotorBalancer.cpp
 *
 *  Created on: Feb 27, 2016
 *      Author: 4917
 */

#include <Components/MotorBalancer.h>
namespace frc4917{
	MotorBalancer::MotorBalancer() {
		difference = 0;
	}

	void MotorBalancer::PIDWrite(double output) {
		difference = output;
	}

	double MotorBalancer::GetDifference() {
		return difference;
	}

	void MotorBalancer::Reset() {
		difference = 0;
	}
}
