#include "BlackMetal.hpp"
#include "log.hpp"

#define PORT 665

INIT_MODULE(BlackMetal);

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

static std::string retrieveAnswer(const std::string &msg)
{
	auto begin = msg.find(':');
	if (begin == std::string::npos) {
		begin = msg.find('=');
		if (begin == std::string::npos) {
			return "";
		}
	}

	auto end = msg.find('}')+1;
	return msg.substr(begin+1, end-1);
}

bm::Status BlackMetal::evalReturnState(const std::string &returnJson)
{
	std::string tmp = retrieveAnswer(returnJson);
	INFO(tmp);

	if (tmp == "RECIEVE_OK") {
		INFO("The execution ran correctly");
		return bm::Status::OK;
	} else {
		ERR("The rebot buffer is full. The send data will not be used.");
		return bm::Status::FULL_BUFFER;
	}
}

