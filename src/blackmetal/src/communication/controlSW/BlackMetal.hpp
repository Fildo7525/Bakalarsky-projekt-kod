#pragma once

#include "RobotDataDelegator.hpp"

#include "controlSW/Odometry.hpp"
#include "RequestMatcher.hpp"

#include <rclcpp/rclcpp.hpp>

/**
 * @class BlackMetal
 * @brief Class for communication with the Blackmetal robot.
 *
 * This class inherits the functionality of Client class
 * and implements the methods for communication with the mentioned Black Metal robot.
 * The left and right wheel velocities are converted from @c geometry_msgs::msg::Twist message type.
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
	[[maybe_unused]] double chassisLength() const;

	/**
	 * @brief Retrieve the wheel radius set from the config file.
	 */
	[[maybe_unused]] double wheelRadius() const;

private:
	/// Object for filtering the duplicated requests.
	std::shared_ptr<RequestMatcher> m_matcher;

	/// Client for sending requests to robot.
	std::shared_ptr<RobotDataDelegator> m_robotDataDelegator;

	/// Object handling the odometry of the robot.
	std::shared_ptr<Odometry> m_odometry;

	/// The angular velocity of left wheel.
	double m_leftWheelSpeed;

	/// The angular velocity of right wheel.
	double m_rightWheelSpeed;

	/// Subscriber that waits for the Twist message and executes the @c onTwistRecievedSendJson callback on it.
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twistSubscriber;

	/// Publisher the odometry of the robot.
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_positionPublisher;
};

