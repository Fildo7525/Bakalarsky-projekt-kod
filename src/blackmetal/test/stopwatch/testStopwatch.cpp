#include <chrono>
#include <gtest/gtest.h>

#include <stopwatch/Stopwatch.hpp>

#include <thread>
#include <unistd.h>

TEST(StopwatchTest, getLastStoppedTime) {
	auto measure = [] (int seconds) -> int {
		TIC;
		std::this_thread::sleep_for(std::chrono::seconds(seconds));
		TOC;
		return Stopwatch::lastStoppedTime() / 1'000'000;
	};

	EXPECT_EQ(measure(1), 1);
	EXPECT_EQ(measure(2), 2);
	EXPECT_EQ(measure(1), 1);
}

TEST(StopwatchTest, getStoppedTimes) {
	auto times = Stopwatch::getStoppedTimes();
	EXPECT_EQ(times.size(), 3ul);
	EXPECT_EQ(int(times[0]/1'000'000), 1);
	EXPECT_EQ(int(times[1]/1'000'000), 2);
	EXPECT_EQ(int(times[2]/1'000'000), 1);
}

TEST(StopwatchTest, maxSizeBounds) {
	for (size_t i = 0; i < 3; i++) {
		{
			Stopwatch s(2);
			std::this_thread::sleep_for(std::chrono::milliseconds(i *200));
		}
	}

	auto times = Stopwatch::getStoppedTimes();
	auto times2 = Stopwatch::getStoppedTimes();

	EXPECT_EQ(times.size(), 2ul);
	EXPECT_EQ(int(times[0]/1'000), 200);
	EXPECT_EQ(int(times[1]/1'000), 400);
	EXPECT_EQ(times2.size(), 0ul);
}

TEST(StopwatchTest, timeAtPosition) {
	for (size_t i = 0; i < 3; i++) {
		{
			Stopwatch s;
			std::this_thread::sleep_for(std::chrono::milliseconds(i *200));
		}
	}

	int one = Stopwatch::stoppedTimeAt(0)/1'000;
	int two = Stopwatch::stoppedTimeAt(1)/1'000;
	int three = Stopwatch::stoppedTimeAt(2)/1'000;

	int four = Stopwatch::stoppedTimeAt(3)/1'000;
	int five = Stopwatch::stoppedTimeAt(-1)/1'000;

	EXPECT_EQ(one, 0);
	EXPECT_EQ(two, 200);
	EXPECT_EQ(three, 400);

	EXPECT_EQ(four, three);
	EXPECT_EQ(four, 400);
	EXPECT_EQ(five, 400);

}

