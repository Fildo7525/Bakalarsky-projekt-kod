# Communication library
add_library(bm_communication
	BlackMetal.cpp
	BlackMetal.hpp
	Client.hpp
	Client.cpp
)

find_package(ament_cmake REQUIRED)

target_include_directories(
	bm_communication
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
)

ament_target_dependencies(bm_communication
	"rclcpp"
	"std_msgs"
	"geometry_msgs"
)

# Logging library
add_library(bm_logging
	BMLogger.cpp
	BMLogger.hpp
	Client.hpp
	Client.cpp
)

target_include_directories(
	bm_logging
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
)

ament_target_dependencies(bm_logging
	"rclcpp"
)
