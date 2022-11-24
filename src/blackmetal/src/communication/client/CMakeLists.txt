# Logging library
add_library(client
	Client.hpp
	Client.cpp
	ReturnStatus.cpp
	ReturnStatus.hpp
)

target_include_directories(
	client
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(client
	logger
	stopwatch
	threadSafeQ
)

