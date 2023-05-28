# Communication library
add_library(dummyServer
	dummyServer.hpp
	dummyServer.cpp
	Pipe.hpp
	Process.hpp
)

target_include_directories(
	dummyServer
PUBLIC
	${CMAKE_SOURCE_DIR}/src/logger
	${CMAKE_SOURCE_DIR}/src/communication
)

target_link_libraries(dummyServer
	logger
	stopwatch
	client
)

