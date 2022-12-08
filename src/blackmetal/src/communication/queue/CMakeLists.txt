# Communication library
add_library(threadSafeQ
	Queue.cpp
	Queue.hpp
)

find_package(ament_cmake REQUIRED)

target_link_libraries(threadSafeQ
	logger
	stopwatch
)

ament_target_dependencies(threadSafeQ
	"rclcpp"
)

