set(FILES
	testStopwatch.cpp
)

add_library(testStopwatch ${FILES})
target_include_directories(testStopwatch
PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(testStopwatch
PUBLIC
	logger
	stopwatch
)

