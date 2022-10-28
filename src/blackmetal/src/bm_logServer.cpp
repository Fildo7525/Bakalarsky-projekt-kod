#include "communication/logSW/BMLogger.hpp"

int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<BMLogger>());

	rclcpp::shutdown();

	return 0;
}

