#pragma once

#include "Client.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <memory>

/**
 * @class BlackMetal
 * @brief Class for communication with the Blackmetal robot.
 *
 * This class inherits the functionality of Client class
 * and implements the methods for communication with the mentioned Black Metal robot.
 * The left and right wheel velocities are converted from geometry_msgs::msg::Twist message type.
 */
class BlackMetal
	: public rclcpp::Node
	, private Client
{
public:
	/// Constructor.
	BlackMetal();

	/**
	 * @brief Convert Twist messages and send them to the Blackmetal robot.
	 *
	 * This function waits for the Twist messages. Transfers them to the json type of string and sends them to the robot.
	 * The linear and angular parameters have to be calculated for wheel for the json.
	 *
	 * @param msg The received message.
	 */
	void onTwistRecievedSendJson(const geometry_msgs::msg::Twist &msg);

private:

	/**
	 * @brief Overridden function from the Client base class.
	 *
	 * The function parses the received json string and converts the result to bm::Status.
	 *
	 * @param returnJson Returned Json string from the server.
	 */
	bm::Status evalReturnState(const std::string &returnJson) override;

private:
	// TODO: Find out what is the chassis length.
	const double M_CHASIS_LENGTH;
	// TODO: Find out what is the wheel radius.
	const double M_WHEEL_RADIUS;

	/// The angular velocity of left wheel.
	double m_leftWheelSpeed;
	/// The angular velocity of right wheel.
	double m_rightWheelSpeed;

	/// Subscriber that waits for the Twist message and executes the onTwistRecievedSendJson callback on it.
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twistSubscriber;
};

