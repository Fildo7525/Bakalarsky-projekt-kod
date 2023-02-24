cmake_minimum_required(VERSION 3.16.3)
project(blackMetal VERSION 3.16.3)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SOURCES
	main.cpp
	initializer/BlackMetal.cpp
	initializer/LowPassFilter.cpp
	initializer/RobotImpulseFilter.cpp
)
set(HEADERS
	initializer/BlackMetal.hpp
	initializer/LowPassFilter.hpp
	initializer/RobotImpulseFilter.hpp
)

add_executable(${PROJECT_NAME}
	${SOURCES}
	${HEADERS}
)

