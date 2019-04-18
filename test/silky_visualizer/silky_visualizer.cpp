#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

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

#define INI_FILENAME "silky_visualizer.ini"

struct iniDataStruct {
	// These are in order that they are expected in the ini file
	bool printPathData;
	double timeStepSize;
	double xZeroPixels;
	double yZeroPixels;
	double mmPerPixel;
	double maxLinearAccel;
	double maxLinearDecel;
	double maxLinearVelocity;
	double maxAngularAccel;
	double xStartPositionMm;
	double yStartPositionMm;
	std::vector<double> positions;
	std::vector<double> angles;
} iniData;


int loadIniData() {
	char line[2048];
	std::ifstream iniFile(INI_FILENAME);

	if (!iniFile) {
		std::cout << "Unable to open ini file " << INI_FILENAME << ".\n";
		return -1;
	}

	// Skip past any comment lines then read single values
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	if (line[0] == '0') {
		iniData.printPathData = false;
	}
	else {
		iniData.printPathData = true;
	}
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.timeStepSize);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.xZeroPixels);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.yZeroPixels);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.mmPerPixel);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.maxLinearAccel);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.maxLinearDecel);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.maxLinearVelocity);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.maxAngularAccel);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.xStartPositionMm);
	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	sscanf(line, "%lf", &iniData.yStartPositionMm);

	// Skip past comment lines and read in position and angle vectors (comma-separated)
	char delimiters[] = ", ";
	char *curNumberStr;
	double curNumber;

	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	curNumberStr = strtok(line, delimiters);
	while(curNumberStr != NULL) {
		sscanf(curNumberStr, "%lf", &curNumber);
		iniData.positions.push_back(curNumber);
		curNumberStr = strtok(NULL, delimiters);
	}

	do { iniFile.getline(line, sizeof(line)); } while (line[0] == '#');
	curNumberStr = strtok(line, delimiters);
	while (curNumberStr != NULL) {
		sscanf(curNumberStr, "%lf", &curNumber);
		iniData.angles.push_back(curNumber);
		curNumberStr = strtok(NULL, delimiters);
	}

	std::cout << "Settings"
			<< "\n--------"
			<< "\nPrint Path =   " << (iniData.printPathData ? "true" : "false")
			<< "\nTime Step =    " << iniData.timeStepSize 
			<< "\nX Zero =       " << iniData.xZeroPixels 
			<< "\nY Zero =       " << iniData.yZeroPixels 
			<< "\nmm Per Pixel = " << iniData.mmPerPixel 
			<< "\nLin Accel =    " << iniData.maxLinearAccel 
			<< "\nLin Decel =    " << iniData.maxLinearDecel
			<< "\nLin Vel =      " << iniData.maxLinearVelocity 
			<< "\nAng Accel =    " << iniData.maxAngularAccel
			<< "\nX Start =      " << iniData.xStartPositionMm 
			<< "\nY Start =      " << iniData.yStartPositionMm
			<< std::endl;

	std::cout << "\nPositions Vector: ";
	for (unsigned int i = 0; i < iniData.positions.size(); i++) {
		std::cout << iniData.positions[i] << ' ';
	}
	std::cout << std::endl;

	std::cout << "Angles Vector:    ";
	for (unsigned int i = 0; i < iniData.angles.size(); i++) {
		std::cout << iniData.angles[i] << ' ';
	}
	std::cout << std::endl;

	return 0;
}

// Change the colour of this pixel
void markLocation(BMP *bmp, int x, int y, unsigned long colour) {
	unsigned int idx = (y * bmp->bmp_info_header.width + x) * (bmp->bmp_info_header.bit_count / 8);

	if (idx > (unsigned int)(bmp->bmp_info_header.width * bmp->bmp_info_header.height * (bmp->bmp_info_header.bit_count / 8))) {
		return;
	}

	bmp->data[idx++] = colour & 0xFF;
	bmp->data[idx++] = (colour >> 8) & 0xFF;
	bmp->data[idx++] = (colour >> 16) & 0xFF;
}

int main() {
	// Load the bitmap (24-bits, 10mm/pixel, alliance wall is near bottom of image, top of image is opposing alliance wall)
	// Used Paint.Net to save image
	if (loadIniData() < 0)
	{
		return -1;
	}
	BMP bmp("Field.bmp");

	// Insert the silky vectors into the next line
	SilkyMotionManager smm(iniData.positions, iniData.angles,
		iniData.maxLinearAccel, iniData.maxLinearDecel, iniData.maxLinearVelocity, iniData.maxAngularAccel);

	if (iniData.printPathData) {
		std::cout << "time, dis, ang, lin_vel, lin_accel, ang_vel, x, y, x_pixel, y_pixel" << std::endl;
	}

	PathInfo lastPi = { 0.0 };
	double lastX = 0.0;
	double lastY = 0.0;
	for (double t = 0; t <= smm.getTotalTime() + 0.03; t += iniData.timeStepSize) {
		PathInfo pi = smm.getCurrentPathInfo(t);

		// Figure out where the robot should be now
		double x = (pi.dis - lastPi.dis) * sin(pi.ang / 180 * M_PI) + lastX;
		double y = (pi.dis - lastPi.dis) * cos(pi.ang / 180 * M_PI) + lastY;;

		if (iniData.printPathData) {
			std::cout << t << ", " << pi.dis << ", " << pi.ang << ", " << pi.lin_vel << ", " << pi.lin_accel << ", " << pi.ang_vel
				<< ", " << x << ", " << y;
		}

		int xLocation = (int)(iniData.xZeroPixels + ((iniData.xStartPositionMm + x) / iniData.mmPerPixel));
		int yLocation = (int)(iniData.yZeroPixels + ((iniData.yStartPositionMm + y) / iniData.mmPerPixel));
		markLocation(&bmp, xLocation, yLocation, 0x00FF0000);

		if (iniData.printPathData) {
			std::cout << ", " << xLocation << ", " << yLocation << std::endl;
		}

		lastPi = pi;
		lastX = x;
		lastY = y;
	}

	bmp.write("Field_with_path.bmp");

	return 0;
}