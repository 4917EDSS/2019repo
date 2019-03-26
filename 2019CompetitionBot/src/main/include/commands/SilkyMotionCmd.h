#ifndef SilkyMotionCmd_H
#define SilkyMotionCmd_H
#include <vector>
#include  "Components/SilkyMotionManager.h"
#include "../Robot.h"

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