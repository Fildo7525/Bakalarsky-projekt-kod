add_library(bm_communication
	BlackMetal.cpp
	BlackMetal.hpp
)

target_include_directories(
	bm_communication
PUBLIC
	../log
)

target_link_libraries(
	bm_communication
PUBLIC
	logger
)

