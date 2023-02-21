#pragma once

#include <string>

class RobotResponseType
{
public:
	RobotResponseType() = default;

	RobotResponseType& setLeftWheel(long leftWheel);
	RobotResponseType& setRightWheel(long rightWheel);

	long leftWheel() const;
	long rightWheel() const;

	std::string toJson() const;
	static RobotResponseType fromJson(const std::string &json);
private:
	double m_leftWheel;
	double m_rightWheel;
};

