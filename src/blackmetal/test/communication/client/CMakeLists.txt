set(FILES
	dummyServer.cpp
	dummyServer.hpp
	Pipe.hpp
	Process.hpp
	testClient.cpp
)

add_library(testClient ${FILES})
target_include_directories(testClient
PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(testClient
PUBLIC
	client
	logger
	stopwatch
	threadSafeQ
)

