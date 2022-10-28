#include "Client.hpp"

#include <rclcpp/rclcpp.hpp>

class BMLogger
	: public rclcpp::Node
	, public Client
{
public:
	/// Constructor.
	BMLogger();

	void onTimerTimeoutReadSocket();
};

