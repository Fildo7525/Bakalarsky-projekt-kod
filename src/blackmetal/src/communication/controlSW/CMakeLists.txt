# Communication library
add_library(bm_controlSW
	BlackMetal.cpp
	BlackMetal.hpp
	../Client.hpp
	../Client.cpp
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
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	target_link_libraries(bm_controlSW
		stopwatch
	)
endif()

ament_target_dependencies(bm_controlSW
	"rclcpp"
	"std_msgs"
	"geometry_msgs"
)

