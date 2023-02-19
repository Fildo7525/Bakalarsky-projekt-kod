#pragma once

#include "communication/client/ReturnStatus.hpp"

#include <iostream>
#include <string>
#include <variant>

class RobotRequestType
{
public:
	/**
	 * The sendRequest function takes arguments for left and right wheel
	 * for setting position and velocity. The velocity is set with double
	 * and the position is set with long. This is the generic solution for this issue.
	 */
	using WheelValueT = std::variant<long, double>;

	RobotRequestType &setUserID(int id);
	RobotRequestType &setCommand(bm::Command cmd);
	RobotRequestType &setLeftWheel(WheelValueT lw);
	RobotRequestType &setRighttWheel(WheelValueT rw);

	int userID();
	bm::Command command();
	WheelValueT leftWheel();
	WheelValueT rightWheel();

	std::string toJson() const;

	bool operator>(const RobotRequestType &other) const;

	friend std::ostream &operator<<(std::ostream &os, const RobotRequestType &request);

private:
	int m_userID;
	bm::Command m_command;
	WheelValueT m_leftWheel;
	WheelValueT m_rightWheel;
};

std::ostream &operator<<(std::ostream &os, const RobotRequestType &request);

