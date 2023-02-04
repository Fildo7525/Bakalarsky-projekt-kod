set(FILES
	testQueue.cpp
)

add_library(testQueue ${FILES})
target_include_directories(testQueue
PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(testQueue
PUBLIC
	logger
	stopwatch
	threadSafeQ
)

