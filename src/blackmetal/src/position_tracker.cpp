#include "log.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/utilities.hpp>

INIT_MODULE(PositionTracker);

/**
 * @class PositionTracker
 * @brief Class for tracking and logging robot location.
 *
 * This class connects to the topic "position" and logs the received messages.
 */
class PositionTracker
	: public rclcpp::Node
{
public:
	PositionTracker()
		: Node("position_tracker")
	{
		m_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
					"odom",
					rclcpp::QoS(10),
					[] (const nav_msgs::msg::Odometry msg) { INFO(msg.pose.pose.position.x << ", " << msg.pose.pose.position.y << ", " << msg.pose.pose.position.z); });
	}
private:
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_subscriber;
};

int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<PositionTracker>());

	rclcpp::shutdown();

	return 0;
}

