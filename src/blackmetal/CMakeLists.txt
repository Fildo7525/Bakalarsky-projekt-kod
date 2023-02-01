cmake_minimum_required(VERSION 3.8)
project(blackmetal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Debug")

include_directories(${CMAKE_SOURCE_DIR}/src)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_subdirectory(src/communication/client)
add_subdirectory(src/communication/controlSW)
add_subdirectory(src/communication/logSW)
add_subdirectory(src/logger)
add_subdirectory(src/stopwatch)






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

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	# the following line skips the linter which checks for copyrights
	# comment the line when a copyright and license is added to all source files
	set(ament_cmake_copyright_FOUND TRUE)
	# the following line skips cpplint (only works in a git repo)
	# comment the line when this package is in a git repo and when
	# a copyright and license is added to all source files
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY
	config
	DESTINATION share/${PROJECT_NAME}/)

ament_package()
