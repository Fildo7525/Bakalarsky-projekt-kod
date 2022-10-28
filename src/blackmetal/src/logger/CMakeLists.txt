add_library(
logger
	colors.hpp
	log.hpp
	Logger.cpp
	Logger.hpp
)

target_include_directories(
	logger
PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}
)

