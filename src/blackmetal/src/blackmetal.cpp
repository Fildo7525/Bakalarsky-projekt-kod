#include "communication/controlSW/BlackMetal.hpp"
#include "log.hpp"
#include <rclcpp/logger.hpp>

INIT_MODULE(bm_main);

int main (int argc, char *argv[])
{
	DBG("Testing debug");
	INFO("Testing info");
	WARN("Testing warn");
	ERR("Testing error");
	FATAL("Testing fatal");
	rclcpp::init(argc, argv);

	rclcpp::spin(std::make_shared<BlackMetal>());

	rclcpp::shutdown();

	return 0;
}

