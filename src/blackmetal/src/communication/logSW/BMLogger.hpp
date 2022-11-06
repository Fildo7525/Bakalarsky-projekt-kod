#pragma once

#include "Client.hpp"

#include <rclcpp/rclcpp.hpp>

class BMLogger
	: public rclcpp::Node
	, private Client
{
public:
	/// Constructor.
	BMLogger();

	/**
	 * @brief A Callback function called every second in the timer to read the data from the socket.
	 *
	 * The calling time is set to one second. However, if the server does not send data
	 * during that period, the function blocks until the server does not send us some string to log.
	 */
	void onTimerTimeoutReadSocket();
private:
	/// Timer calling the defined callback function.
	rclcpp::TimerBase::SharedPtr m_timer;
};

