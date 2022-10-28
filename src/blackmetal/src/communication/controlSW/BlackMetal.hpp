#pragma once

#include "../Client.hpp"

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

/**
 * @class BlackMetal
 * @brief Singleton class for communication with the blackmetal robot.
 *
 * The class creates client and returns this instance on every call.
 */
class BlackMetal
	: public rclcpp::Node
	, private Client
{
public:
	/// Constructor.
	BlackMetal();

	/// Destructor.
	~BlackMetal();

	/**
	 * @brief Convert Twist messages and send them to the Blackmetal robot.
	 *
	 * This function wates for the Twist messages. Transfers them to the json type of string and sends them to the robot.
	 * The linear and angular parameters have to be calculated for wheel for the json.
	 *
	 * @param msg The received message.
	 */
	void onTwistRecievedSendJson(const geometry_msgs::msg::Twist &msg);

private:

	/**
	 * @brief Overriden function from the Client base class.
	 *
	 * @param returnJson Returned Json string from the server.
	 */
	bm::Status evalReturnState(const std::string &returnJson) override;

private:
	const double M_CHASIS_LENGTH;

	double m_leftWheelSpeed;
	double m_rightWheelSpeed;

	/// Subscriber that wates for the Twist message and executes the onTwistRecievedSendJson callback on it.
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twistSubscriber;
};

