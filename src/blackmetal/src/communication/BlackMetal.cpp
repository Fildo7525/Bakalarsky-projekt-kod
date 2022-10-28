#include "BlackMetal.hpp"
#include "log.hpp"

#define PORT 665

BlackMetal::BlackMetal()
	: rclcpp::Node("blackmetal")
	, Client(PORT, "192.168.1.3")
	, M_CHASIS_LENGTH(0.45)
{
	m_twistSubscriber
		= this->create_subscription<geometry_msgs::msg::Twist>(
			"bm_movement",
			1,
			[this] (const geometry_msgs::msg::Twist &msg) {
				this->onTwistRecievedSendJson(msg);
			}
	);
}

BlackMetal::~BlackMetal()
{
}

void BlackMetal::onTwistRecievedSendJson(const geometry_msgs::msg::Twist &msg)
{
	DBG("Message geometry_msgs::msg::Twist: " << geometry_msgs::msg::to_yaml(msg));
	m_rightWheelSpeed = msg.linear.y + (M_CHASIS_LENGTH * msg.angular.z)/2;
	m_leftWheelSpeed = msg.linear.y - (M_CHASIS_LENGTH * msg.angular.z)/2;
	request(m_rightWheelSpeed, m_leftWheelSpeed);
}

