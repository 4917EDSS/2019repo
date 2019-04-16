#include "../../2019CompetitionBot/src/main/include/components/SilkyMotionManager.h"
#include <vector>
#include <iostream>
#include <math.h>
// Run this command line in a Unix-like shell on Windows (e.g. MSYS2)
// g++ -std=c++11 silky_test.cpp ../../2019CompetitionBot/src/main/cpp/components/SilkyMotionManager.cpp -I../../2019CompetitionBot/src/main/include -o silky_test.exe

// Redirect the output to a .csv file and you'll be able to plot x, y to visualize the path
// e.g. 
//   From 2019repo/test/silky_test, in an MSYS terminal, run
//     silky_test.exe > silky.csv

// To install MSYS2 64-bit for Windows
//		- Install from https://www.msys2.org/
//		- Start MSYS2 and run these commands to get the required packages
//			pacman -Syu
//				('Y' to proceed with installation)
//				(when prompted, close MSYS2 without returning to the prompt, then re-open)
//			pacman -Su
//				('Y' to proceed with installation)

int main() {
	
  // Insert the silky vectors into the next line
  SilkyMotionManager smm(std::vector<double> {550, -550, 4050, 2000}, std::vector<double> {0, 0, -30, 120}, 
						MAX_LIN_ACCEL, MAX_LIN_DECEL, MAX_LIN_VEL, MAX_ANG_ACCEL);
						
  std::cout << "time, dis, ang, lin_vel, lin_accel, ang_vel, x, y" << std::endl;

  PathInfo lastPi = {0.0};
  double lastX = 0.0;
  double lastY = 0.0;
  for (double t = 0; t <= smm.getTotalTime()+0.03; t += 0.02) {
    PathInfo pi = smm.getCurrentPathInfo(t);
	double x = (pi.dis - lastPi.dis) * sin(pi.ang/180 * M_PI) + lastX;
	double y = (pi.dis - lastPi.dis) * cos(pi.ang/180 * M_PI) + lastY;;
    
	std::cout << t << ", " << pi.dis << ", " << pi.ang << ", " << pi.lin_vel << ", " << pi.lin_accel << ", " << pi.ang_vel 
			  << ", " << x << ", " << y << std::endl;
	
	lastPi = pi;
	lastX = x;
	lastY = y;
  }
}
