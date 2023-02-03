#include <gtest/gtest.h>

#include <stopwatch/Stopwatch.hpp>

#include <unistd.h>

TEST(StopwatchTest, getLastStoppedTime) {
	auto measure = [] (int seconds) -> int {
		TIC;
		sleep(seconds);
		TOC;
		return Stopwatch::lastStoppedTime() / 1'000'000;
	};

	EXPECT_EQ(measure(1), 1);
	EXPECT_EQ(measure(2), 2);
	EXPECT_EQ(measure(3), 3);
}

