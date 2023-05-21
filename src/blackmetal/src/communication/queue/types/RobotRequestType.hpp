#pragma once

#include "communication/client/ReturnStatus.hpp"

#include <variant>
#include <chrono>

/**
 * @class RobotRequestType
 * @brief Class for storing and managing the request parameters.
 *
 * This class is used for storing the request parameters and their transformation to json string.
 */
class RobotRequestType
{
public:
	/**
	 * @brief Default constructor.
	 */
	RobotRequestType()
		: m_time(std::chrono::system_clock::now())
	{}

	/**
	 * The sendRequest function takes arguments for left and right wheel
	 * for setting position and velocity. The velocity is set with double
	 * and the position is set with long. This is the generic solution for this issue.
	 */
	using WheelValueT = std::variant<long, double>;

	/**
	 * Sets the user ID.
	 *
	 * @param id User ID to be set in the request.
	 * @return RobotRequestType &Reference to this object.
	 */
	RobotRequestType &setUserID(int id);

	/**
	 * Sets the command.
	 *
	 * @param cmd Command to be set in the request.
	 * @return RobotRequestType &Reference to this object.
	 */
	RobotRequestType &setCommand(bm::Command cmd);

	/**
	 * Sets the left wheel value.
	 *
	 * @param lw Left wheel value to be set in the request. This applies only
	 * in @c bm::Command::SET_LR_WHEEL_VELOCITY and @c bm::Command::SET_LR_WHEEL_POSITION.
	 * @return RobotRequestType &Reference to this object.
	 */
	RobotRequestType &setLeftWheel(WheelValueT lw);

	/**
	 * Sets the right wheel value.
	 *
	 * @param rw Right wheel value to be set in the request. This applies only
	 * in @c bm::Command::SET_LR_WHEEL_VELOCITY and @c bm::Command::SET_LR_WHEEL_POSITION.
	 * @return RobotRequestType &Reference to this object.
	 */
	RobotRequestType &setRightWheel(WheelValueT rw);

	/**
	 * Returns the user ID.
	 *
	 * @return int User ID.
	 */
	int userID() const;

	/**
	 * Returns the request command.
	 */
	bm::Command command() const;

	/**
	 * Returns the left wheel value.
	 *
	 * @return @c WheelValueT Left wheel value.
	 */
	WheelValueT leftWheel() const;

	/**
	 * Returns the right wheel value.
	 *
	 * @return @c WheelValueT Right wheel value.
	 */
	WheelValueT rightWheel() const;

	/**
	 * Forms a json string out of the request.
	 */
	std::string toJson() const;

	/**
	 * @brief Compares two requests.
	 *
	 * The requests are compared by their command. The lower command numbers
	 * are @c EMEERGENCY_STOP, @c SET_LR_WHEEL_VELOCITY, @c SET_LR_WHEEL_POSITION.
	 * When requests with this enqueued they have priority over the other requests.
	 *
	 * @param other The other request to compare with.
	 * @return true When the this request has higher priority.
	 */
	bool operator>(const RobotRequestType &other) const;

	friend std::ostream &operator<<(std::ostream &os, const RobotRequestType &request);

private:
	/// Timestamp used for comparing the order of request in @c ts::Queue.
	std::chrono::system_clock::time_point m_time;

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

