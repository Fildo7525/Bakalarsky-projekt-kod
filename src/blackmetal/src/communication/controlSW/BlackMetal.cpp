#include "BlackMetal.hpp"

#include "colors.hpp"
#include "log.hpp"

#define PORT 665

INIT_MODULE(BlackMetal);

BlackMetal::BlackMetal()
	: rclcpp::Node("blackmetal")
	, Client(PORT, "192.168.1.3")
	, M_CHASIS_LENGTH(0.45)
	, M_WHEEL_RADIUS(0.1)
{
	this->declare_parameter("bm_csIP", "192.168.1.3");
	INFO("Client connected. Continuing the parameter initialization");

	m_twistSubscriber
		= this->create_subscription<geometry_msgs::msg::Twist>(
			"bm_movement",
			1,
			[this] (const geometry_msgs::msg::Twist &msg) {
				this->onTwistRecievedSendJson(msg);
			}
	);
}

void BlackMetal::onTwistRecievedSendJson(const geometry_msgs::msg::Twist &msg)
{
	DBG("Message geometry_msgs::msg::Twist: " << geometry_msgs::msg::to_yaml(msg));
	m_rightWheelSpeed = (msg.linear.x + 0.5 * M_CHASIS_LENGTH * msg.angular.z)/M_WHEEL_RADIUS;
	m_leftWheelSpeed = (msg.linear.x - 0.5 * M_CHASIS_LENGTH * msg.angular.z)/M_WHEEL_RADIUS;
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

