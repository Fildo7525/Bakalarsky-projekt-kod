#include "Client.hpp"

#include <rclcpp/rclcpp.hpp>

class BMLogger
	: public rclcpp::Node
	, private Client
{
public:
	/// Constructor.
	BMLogger();

	void onTimerTimeoutReadSocket();
private:
	bm::Status evalReturnState(const std::string &returnJson) override;
};

