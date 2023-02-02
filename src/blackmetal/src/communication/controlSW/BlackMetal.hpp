#pragma once

#include "Client.hpp"

#include "controlSW/Odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <string>
#include <thread>

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

	/**
	 * @brief Retrieve the chassis length set in the config file.
	 */
	const double &chassisLength();

	/**
	 * @brief Retrieve the wheel radius set from the config file.
	 */
	const double &wheelRadius();

private:
	/// Client for sending requests to robot.
	std::shared_ptr<Client> m_controlClient;
	/// Object handling the odometry of the robot.
	std::shared_ptr<Odometry> m_odometry;

	/// The angular velocity of left wheel.
	double m_leftWheelSpeed;
	/// The angular velocity of right wheel.
	double m_rightWheelSpeed;

	/// Subscriber that waits for the Twist message and executes the onTwistRecievedSendJson callback on it.
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twistSubscriber;
};

