#include "BlackMetal.hpp"

#include "colors.hpp"
#include "log.hpp"

#define PORT 665

INIT_MODULE(BlackMetal);

BlackMetal::BlackMetal()
	: rclcpp::Node("blackmetal")
	, Client(PORT, "192.168.1.68")
	, M_CHASIS_LENGTH(0.45)
	, M_WHEEL_RADIUS(0.1)
{
	INFO("Client connected. Continuing the parameter initialization");

	auto preparedState = execute(bm::Command::PREPARE_WHEEL_CONTROLLER);
	const bm::Status *status = std::get_if<bm::Status>(&preparedState);

	if (status != nullptr) {
		ERR("The robot returned an error: " << Client::stringifyStatus(*status));
	} else {
		INFO("The robot succeeded with message: " << std::get<std::string>(preparedState));
	}

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
	INFO("\033[32;1m" << returnJson << RESET);

	if (returnJson.find("RECIEVE_OK") == std::string::npos) {
		ERR("The rebot buffer is full. The send data will not be used.");
		return bm::Status::FULL_BUFFER;
	}

	INFO("The execution ran correctly");
	return bm::Status::OK;
}

