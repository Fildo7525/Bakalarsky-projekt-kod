#include "log.hpp"
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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
		m_subscriber = this->create_subscription<geometry_msgs::msg::Vector3>(
					"position",
					rclcpp::QoS(10),
					[] (const geometry_msgs::msg::Vector3 msg) { INFO(msg.x << ", " << msg.y << ", " << msg.z); });
	}
private:
	rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_subscriber;
};

int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<PositionTracker>());

	rclcpp::shutdown();

	return 0;
}

