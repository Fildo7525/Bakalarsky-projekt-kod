#include "controlSW/Odometry.hpp"
#include "queue/types/RobotRequestType.hpp"

class TestOdometry
	: public Odometry
{
public:
	double testWrapAngle(double angle);
private:
	
};
