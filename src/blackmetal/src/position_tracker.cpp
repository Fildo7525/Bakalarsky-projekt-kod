#include "log.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/vector3.hpp>

INIT_MODULE(PositionTracker);

void callback(geometry_msgs::msg::Vector3 msg)
{
	INFO(msg.x << ", " << msg.y << ", " << msg.z);
}

int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	auto node = rclcpp::Node::make_shared("position");

	node->create_subscription<geometry_msgs::msg::Vector3>("position", 1, &callback);

	return 0;
}

