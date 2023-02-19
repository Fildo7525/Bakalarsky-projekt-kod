# Communication library
add_library(threadSafeQ
	Queue.hpp
	types/RobotRequestType.cpp
	types/RobotRequestType.hpp
)

find_package(ament_cmake REQUIRED)

target_include_directories(
	threadSafeQ
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(threadSafeQ
	logger
	stopwatch
)

ament_target_dependencies(threadSafeQ
	"rclcpp"
)

