#pragma once

#include "Client.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @class BMLogger
 * @brief This class represents a client, which connects to robot and handles the messages that are handed to him as logs.
 * The received messages are in JSON format. These messages contain two key
 * @li state
 * @li direction
 * Both of these keys have values represented by integers.
 *
 * NOTE: This is written in robot's documentation. However the robot only sends STATE_READY
 * and DIRECTION_NORTH. No other value is sent.
 *
 * The state can have five states:
 * @li 0: STATE_UNKNOWN
 * @li 1: STATE_READY
 * @li 2: STATE_ERROR
 * @li 3: STATE_RUN
 * @li 4: STATE_SHUTDOWN
 *
 * The direction can have four states:
 * @li 1: DIRECTION_NORTH
 * @li 2: DIRECTION_SOUTH
 * @li 3: DIRECTION_WEST
 * @li 4: DIRECTION_EAST
 */
class BMLogger
	: public rclcpp::Node
{
public:
	/// Constructor.
	BMLogger();

	/**
	 * @brief A Callback function called every second in the timer to read the data from the socket.
	 *
	 * The calling time is set to one second. However, if the server does not send data
	 * during that period, the function blocks until the server does not send us some
	 * string for too log.
	 */
	void onTimerTimeoutReadSocket();

private:
	/// Timer calling the defined callback function.
	rclcpp::TimerBase::SharedPtr m_timer;

	/// Client communicating with the logging server of the robot.
	std::shared_ptr<Client> m_client;
};

