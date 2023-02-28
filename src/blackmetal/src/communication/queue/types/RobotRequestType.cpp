#include "RobotRequestType.hpp"

#include <log.hpp>

INIT_MODULE(RobotRequestType);

RobotRequestType& RobotRequestType::setUserID(int id)
{
	this->m_userID = id;
	return *this;
}

RobotRequestType& RobotRequestType::setCommand(bm::Command cmd)
{
	this->m_command = cmd;
	return *this;
}

RobotRequestType& RobotRequestType::setLeftWheel(WheelValueT lw)
{
	this->m_leftWheel = lw;
	return *this;
}

RobotRequestType& RobotRequestType::setRighttWheel(WheelValueT rw)
{
	this->m_rightWheel = rw;
	return *this;
}

int RobotRequestType::userID() const
{
	return m_userID;
}

bm::Command RobotRequestType::command() const
{
	return m_command;
}

RobotRequestType::WheelValueT RobotRequestType::leftWheel() const
{
	return m_leftWheel;
}

RobotRequestType::WheelValueT RobotRequestType::rightWheel() const
{
	return m_rightWheel;
}

std::string RobotRequestType::toJson() const
{
	// Example: "{\"UserID\":1,\"Command\":3,\"RightWheelSpeed\":0.1,\"LeftWheelSpeed\":0.1,\"RightWheelPosition\":0.1,\"LeftWheelPosition\":0.1}";
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(m_command));
	if (m_command == bm::Command::SET_LR_WHEEL_VELOCITY) {
		message += ",\"RightWheelSpeed\":" + std::to_string(std::get<double>(m_rightWheel)) +
					",\"LeftWheelSpeed\":" + std::to_string(std::get<double>(m_leftWheel));
	} else if (m_command == bm::Command::SET_LR_WHEEL_POSITION) {
		message += ",\"RightWheelPosition\":" + std::to_string(std::get<long>(m_rightWheel)) +
					",\"LeftWheelPosition\":" + std::to_string(std::get<long>(m_leftWheel));
	}
	message += "}";

	INFO("sending: " << message);
	return message;
}

bool RobotRequestType::operator>(const RobotRequestType &other) const
{
	// The lower the command, the higher the priority.
	return m_command < other.m_command;
}

std::ostream& operator<<(std::ostream &os, const RobotRequestType &request)
{
	return os << request.toJson();
}

