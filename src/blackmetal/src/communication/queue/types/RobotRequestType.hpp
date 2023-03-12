#pragma once

#include "communication/client/ReturnStatus.hpp"

#include <iostream>
#include <string>
#include <variant>

/**
 * @class RobotRequestType
 * @brief Class for storing the request parameters.
 *
 * This class is used for storing the request parameters and their transformation to json string.
 */
class RobotRequestType
{
public:
	/**
	 * @brief Default constructor.
	 */
	RobotRequestType() = default;

	/**
	 * The sendRequest function takes arguments for left and right wheel
	 * for setting position and velocity. The velocity is set with double
	 * and the position is set with long. This is the generic solution for this issue.
	 */
	using WheelValueT = std::variant<long, double>;

	/**
	 * @brief Sets the user ID.
	 *
	 * @param id User ID to be set in the request.
	 * @return RobotRequestType& Reference to the object.
	 */
	RobotRequestType &setUserID(int id);

	/**
	 * @brief Sets the command.
	 *
	 * @param cmd Command to be set in the request.
	 * @return RobotRequestType& Reference to the object.
	 */
	RobotRequestType &setCommand(bm::Command cmd);

	/**
	 * @brief Sets the left wheel value.
	 *
	 * @param lw Left wheel value to be set in the request. This applies only in bm::Command::SET_LR_WHEEL_VELOCITY and bm::Command::SET_LR_WHEEL_POSITION.
	 * @return RobotRequestType& Reference to the object.
	 */
	RobotRequestType &setLeftWheel(WheelValueT lw);

	/**
	 * @brief Sets the right wheel value.
	 *
	 * @param rw Right wheel value to be set in the request. This applies only in bm::Command::SET_LR_WHEEL_VELOCITY and bm::Command::SET_LR_WHEEL_POSITION.
	 * @return RobotRequestType& Reference to the object.
	 */
	RobotRequestType &setRightWheel(WheelValueT rw);

	/**
	 * @brief Returns the user ID.
	 *
	 * @return int User ID.
	 */
	int userID() const;

	/**
	 * @brief Returns the command.
	 */
	bm::Command command() const;

	/**
	 * @brief Returns the left wheel value.
	 *
	 * @return WheelValueT Left wheel value.
	 */
	WheelValueT leftWheel() const;

	/**
	 * @brief Returns the right wheel value.
	 *
	 * @return WheelValueT Right wheel value.
	 */
	WheelValueT rightWheel() const;

	/**
	 * @brief Forms a json string out of the request.
	 */
	std::string toJson() const;

	/**
	 * @brief Compares two requests.
	 *
	 * The requests are compared by their command. The lower command numbers
	 * are EMEERGENCY_STOP, SET_LR_WHEEL_VELOCITY, SET_LR_WHEEL_POSITION.
	 * When requests with this enqueued they have priority over the other requests.
	 *
	 * @param other The other request to compare with.
	 * @return true When the this request has higher priority.
	 */
	bool operator>(const RobotRequestType &other) const;

	friend std::ostream &operator<<(std::ostream &os, const RobotRequestType &request);

private:
	/// User ID of the request.
	int m_userID;

	/// Command that the robot should execute.
	bm::Command m_command;

	/// Left wheel value of the request. Applies only in bm::Command::SET_LR_WHEEL_VELOCITY and bm::Command::SET_LR_WHEEL_POSITION.
	WheelValueT m_leftWheel;

	/// Right wheel value of the request. Applies only in bm::Command::SET_LR_WHEEL_VELOCITY and bm::Command::SET_LR_WHEEL_POSITION.
	WheelValueT m_rightWheel;
};

std::ostream &operator<<(std::ostream &os, const RobotRequestType &request);

