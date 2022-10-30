#include "BlackMetal.hpp"
#include "log.hpp"

#define PORT 665

INIT_MODULE(BlackMetal);

BlackMetal::BlackMetal()
	: rclcpp::Node("blackmetal")
	, Client(PORT, "192.168.1.3")
	, M_CHASIS_LENGTH(0.45)
	, M_WHEEL_RADIUS(0.1)
{
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

static std::string retrieveAnswer(const std::string &msg)
{
	auto offset = msg.find(':');
	if (offset == std::string::npos) {
		offset = msg.find('=');
		if (offset == std::string::npos) {
			return "";
		}
	}

	auto range = msg.find('}') - 3 - offset;
	WARN("Beigning is calculated to " << offset << ". End of recieved stirng is " << range << ". Length: " << msg.length());
	return msg.substr(offset+2, range);
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

