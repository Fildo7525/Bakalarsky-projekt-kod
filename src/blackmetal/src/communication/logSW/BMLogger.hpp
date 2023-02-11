#pragma once

#include "Client.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>

/**
 * @class BMLogger
 * @brief This class represents a client, which connects to robot and handles the messages that are handed to him as logs.
 * The received messages are in JSON format. These messages contain two key
 *  - state
 *  - direction
 * Both of these keys have values represented by integers.
 *
 * TODO: In the interview ask about the state and direction and their meaning.
 *
 * The state can have five states:
 *  - 0: STATE_UNKNOWN
 *  - 1: STATE_READY
 *  - 2: STATE_ERROR
 *  - 3: STATE_RUN
 *  - 4: STATE_SHUTDOWN
 *
 * The direction can have four states:
 *  - 1: DIRECTION_NORTH
 *  - 2: DIRECTION_SOUTH
 *  - 3: DIRECTION_WEST
 *  - 4: DIRECTION_EAST
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
	 * during that period, the function blocks until the server does not send us some string for too log.
	 *
	 * TODO: Ask about the rate of the messages. If they are send at 1Hz (faster/slower)
	 * Based on their meaning change the logging level and do some actions.
	 */
	void onTimerTimeoutReadSocket();
private:
	/// Timer calling the defined callback function.
	rclcpp::TimerBase::SharedPtr m_timer;
	
	/// Client communicating with the logging server of the robot.
	std::shared_ptr<Client> m_client;
};

