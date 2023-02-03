set(TEST_FILES
	main.cpp
	stopwatch/testStopwatch.cpp
	communication/queue/testQueue.cpp
)

find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()

ament_add_gtest(${PROJECT_NAME}_test ${TEST_FILES})

target_link_libraries(${PROJECT_NAME}_test
	stopwatch
	threadSafeQ
)

install(TARGETS
	${PROJECT_NAME}_test
	DESTINATION lib/${PROJECT_NAME}
)

