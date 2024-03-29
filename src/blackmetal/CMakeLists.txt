cmake_minimum_required(VERSION 3.8)
project(blackmetal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

include_directories(${CMAKE_SOURCE_DIR}/src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)

add_subdirectory(src/logger)
add_subdirectory(src/stopwatch)

add_subdirectory(src/communication/queue)
add_subdirectory(src/communication/client)
add_subdirectory(src/communication/controlSW)
add_subdirectory(src/communication/logSW)






# Main Program
add_executable(blackmetal src/blackmetal.cpp)
target_link_libraries(blackmetal
	bm_controlSW
)
target_include_directories(
	blackmetal
PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(blackmetal
	"rclcpp"
	"std_msgs"
	"geometry_msgs"
	"nav_msgs"
)

install(TARGETS blackmetal
	DESTINATION lib/${PROJECT_NAME})







# Logging server
add_executable(bm_logger ./src/bm_logServer.cpp)
target_include_directories(
	bm_logger
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
target_link_libraries(bm_logger
	bm_logSW
)
ament_target_dependencies(bm_logger
	"rclcpp"
)

install(TARGETS bm_logger
	DESTINATION lib/${PROJECT_NAME})






# Position Tracker
add_executable(bm_position src/position_tracker.cpp)
target_link_libraries(bm_position
	logger
)
target_include_directories(
	bm_position
PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

ament_target_dependencies(bm_position
	"rclcpp"
	"geometry_msgs"
	"nav_msgs"
)

install(TARGETS bm_position
	DESTINATION lib/${PROJECT_NAME})



# TESTING
if(BUILD_TESTING)
	add_subdirectory(test)
endif()

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME}/)

ament_package()
