#include <Queue.hpp>

#include <gtest/gtest.h>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

TEST(QueueTest, fillAndEmpty) {
	ts::Queue<std::string> q("q");

	q.push("1");
	q.push("2");
	q.push("3");

	EXPECT_EQ(q.size(), 3ul);

	q.pop();
	q.pop();
	q.pop();

	EXPECT_EQ(q.size(), 0ul);
}

TEST(QueueTest, multitreadedFillAndEmpty) {
	ts::Queue <std::string> q("q");
	
	std::thread t([&] {
		std::this_thread::sleep_for(1s);
		q.push("1");
		q.push("2");
		q.push("3");
	});


	q.pop();
	q.pop();
	q.pop();

	t.join();

	EXPECT_EQ(q.size(), 0ul);
}

