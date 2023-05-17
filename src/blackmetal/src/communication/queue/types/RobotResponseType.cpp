#include "RobotResponseType.hpp"

RobotResponseType &RobotResponseType::setLeftWheel(long leftWheel)
{
	m_leftWheel = leftWheel;
	return *this;
}

RobotResponseType &RobotResponseType::setRightWheel(long rightWheel)
{
	m_rightWheel = rightWheel;
	return *this;
}

long RobotResponseType::leftWheel() const
{
	return m_leftWheel;
}

long RobotResponseType::rightWheel() const
{
	return m_rightWheel;
}

std::string RobotResponseType::toJson() const
{
	return std::string("{\"LeftWheelSpeed\":") + std::to_string(m_leftWheel) + ",\"RightWheelSpeed\":" + std::to_string(m_rightWheel) + "}";
}

RobotResponseType RobotResponseType::fromJson(const std::string &json)
{
	// The structure will arrive in a wannabe json format
	// {"LeftWheelSpeed"=%ld "RightWheelSpeed"=%ld}
	double lws, rws;

	auto lws_start = json.find_first_of('=') + 1;
	auto lws_end = json.find_first_of(' ');
	lws = std::stod(json.substr(lws_start, lws_end));

	auto rws_start = json.find_last_of('=') + 1;
	auto rws_end = json.find_last_of('}');
	rws = std::stod(json.substr(rws_start, rws_end));

	return RobotResponseType().setLeftWheel(lws).setRightWheel(rws);
}

bool RobotResponseType::operator>(const RobotResponseType &other) const
{
	// The response type must be ordered chrobologically.
	(void)other;
	return false;
}

std::ostream &operator<<(std::ostream &os, const RobotResponseType &robotResponse)
{
	return os << robotResponse.toJson();
}

