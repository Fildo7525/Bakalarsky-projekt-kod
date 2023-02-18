# Communication library
add_library(bm_controlSW
	BlackMetal.cpp
	BlackMetal.hpp
	Odometry.cpp
	Odometry.hpp
	filter/LowPassFilter.cpp
	filter/LowPassFilter.hpp
	filter/RobotImpulseFilter.cpp
	filter/RobotImpulseFilter.hpp
)

find_package(ament_cmake REQUIRED)

target_include_directories(
	bm_controlSW
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
	${CMAKE_SOURCE_DIR}/src/communication
)
target_link_libraries(bm_controlSW
	logger
	stopwatch
	client
)

ament_target_dependencies(bm_controlSW
	"rclcpp"
	"std_msgs"
	"geometry_msgs"
)

