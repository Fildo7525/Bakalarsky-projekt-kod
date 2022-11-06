#include "communication/controlSW/BlackMetal.hpp"
#include "log.hpp"
#include <rclcpp/logger.hpp>

int main (int argc, char *argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<BlackMetal>());

	rclcpp::shutdown();

	return 0;
}

