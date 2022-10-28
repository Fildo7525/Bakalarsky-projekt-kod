# Logging library
add_library(bm_logSW
	BMLogger.cpp
	BMLogger.hpp
	../Client.hpp
	../Client.cpp
)

target_include_directories(
	bm_logSW
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
)

target_link_libraries(bm_logSW
	logger
)

ament_target_dependencies(bm_logSW
	"rclcpp"
)
