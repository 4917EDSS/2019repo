#include <vector>
#include <iostream>
#include <math.h>

#include "bmp.h"
#include "../../2019CompetitionBot/src/main/include/components/SilkyMotionManager.h"

// Run this command line in a Unix-like shell on Windows (e.g. MSYS2)
// g++ -std=c++11 silky_visualizer.cpp ../../2019CompetitionBot/src/main/cpp/components/SilkyMotionManager.cpp -I../../2019CompetitionBot/src/main/include -o silky_visualizer.exe

// To install MSYS2 64-bit for Windows
//		- Install from https://www.msys2.org/
//		- Start MSYS2 and run these commands to get the required packages
//			pacman -Syu
//				('Y' to proceed with installation)
//				(when prompted, close MSYS2 without returning to the prompt, then re-open)
//			pacman -Su
//				('Y' to proceed with installation)


//#define PRINT_DATA

// Where the bottom left corner of the field is, in pixels
#define FIELD_BOTTOM_LEFT_X 204
#define FIELD_BOTTOM_LEFT_Y 243

// Change the colour of this pixel
void markLocation(BMP *bmp, int x, int y, unsigned long colour) {
	unsigned int idx = (y * bmp->bmp_info_header.width + x) * (bmp->bmp_info_header.bit_count / 8);
	
	if(idx > (unsigned int)(bmp->bmp_info_header.width * bmp->bmp_info_header.height * (bmp->bmp_info_header.bit_count / 8))) {
		return;
	}
	
	bmp->data[idx++] = colour & 0xFF;
	bmp->data[idx++] = (colour >> 8) & 0xFF;
	bmp->data[idx++] = (colour >> 16) & 0xFF;
}

int main() {
	// Load the bitmap (24-bits, 10mm/pixel, alliance wall is near bottom of image, top of image is opposing alliance wall)
	// Used Paint.Net to save image
	BMP bmp("Field.bmp");
	
	// Specify starting position in mm from bottom left corner of the field (not the image)
	// This is the centre of the robot.
	int xStart = 3020;
	int yStart = 920;

	// Insert the silky vectors into the next line
	SilkyMotionManager smm(std::vector<double> {550, 4050, 2000}, std::vector<double> {0, -30, 120}, 
			MAX_LIN_ACCEL, MAX_LIN_DECEL, MAX_LIN_VEL, MAX_ANG_ACCEL);
	
#ifdef PRINT_DATA
	std::cout << "time, dis, ang, lin_vel, lin_accel, ang_vel, x, y" << std::endl;
#endif

	PathInfo lastPi = {0.0};
	double lastX = 0.0;
	double lastY = 0.0;
	for (double t = 0; t <= smm.getTotalTime()+0.03; t += 0.02) {
		PathInfo pi = smm.getCurrentPathInfo(t);

		// Figure out where the robot should be now
		double x = (pi.dis - lastPi.dis) * sin(pi.ang/180 * M_PI) + lastX;
		double y = (pi.dis - lastPi.dis) * cos(pi.ang/180 * M_PI) + lastY;;

#ifdef PRINT_DATA
		std::cout << t << ", " << pi.dis << ", " << pi.ang << ", " << pi.lin_vel << ", " << pi.lin_accel << ", " << pi.ang_vel 
		<< ", " << x << ", " << y << std::endl;
#endif

		markLocation(&bmp, FIELD_BOTTOM_LEFT_X + (xStart + x)/10, FIELD_BOTTOM_LEFT_Y + (yStart + y)/10, 0x00FF0000);

		lastPi = pi;
		lastX = x;
		lastY = y;
	}

	bmp.write("Field_with_path.bmp");

	return 0;
}