#include "BlackMetal.hpp"

#include "colors.hpp"
#include "log.hpp"

#define PORT 665

INIT_MODULE(BlackMetal);

BlackMetal::BlackMetal()
	: rclcpp::Node("blackmetal")
	, Client()
{
	m_chasisLength = declare_parameter<double>("chasis", 1);
	m_wheelRadius = declare_parameter<double>("wheelRadius", 0.2);
	this->start(PORT, declare_parameter("bm_csIP", "192.168.1.3"));

	m_twistSubscriber
		= this->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel",
			1,
			[this] (const geometry_msgs::msg::Twist &msg) {
				this->onTwistRecievedSendJson(msg);
			}
	);
	// TODO: Create a service for executing the other commands.
}

void BlackMetal::onTwistRecievedSendJson(const geometry_msgs::msg::Twist &msg)
{
	DBG("Message geometry_msgs::msg::Twist: " << geometry_msgs::msg::to_yaml(msg));
	m_rightWheelSpeed = (msg.linear.x + 0.5 * m_chasisLength * msg.angular.z)/m_wheelRadius;
	m_leftWheelSpeed = (msg.linear.x - 0.5 * m_chasisLength * msg.angular.z)/m_wheelRadius;
	request(m_rightWheelSpeed, m_leftWheelSpeed);
}

bm::Status BlackMetal::evalReturnState(const std::string &returnJson)
{
	if (returnJson.find("RECIEVE_OK") == std::string::npos) {
		WARN("The robot buffer is full. The send data will not be used: " << returnJson);
		return bm::Status::FULL_BUFFER;
	}

	SUCCESS(returnJson);
	return bm::Status::OK;
}

