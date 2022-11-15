#pragma once

class Odometry;
#include "controlSW/BlackMetal.hpp"

#include <rclcpp/timer.hpp>

extern std::mutex odometryMutex;

class Odometry
{
public:
	struct speed
	{
		long leftWheel;
		long rightWheel;
	};

	explicit Odometry(BlackMetal &controlSoftware);
	void execute();
	speed obtainWheelSpeeds(const std::string &jsonMessage);

	long leftWheelSpeed() const;
	long rightWheelSpeed() const;
private:
	BlackMetal &m_controlSoftware;
	rclcpp::TimerBase::SharedPtr m_timer;

	// The speeds in the blackmetal code are defined as longs.
	long m_leftWheel;
	long m_rightWheel;
};

