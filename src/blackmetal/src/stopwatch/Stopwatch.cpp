#include "Stopwatch.hpp"

#include <chrono>
#include <mutex>

static std::vector stoppedTimes = std::vector<double>();

std::mutex mut;

Stopwatch::Stopwatch(size_t maxLength)
	: m_start(std::chrono::high_resolution_clock::now())
	, m_maxLength(maxLength)
{
}

Stopwatch::~Stopwatch()
{
	auto diff = std::chrono::high_resolution_clock::now() - m_start;
	double diffMiliseconds = std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
	{
		std::lock_guard lock(mut);
		if (m_maxLength == stoppedTimes.size()) {
			stoppedTimes.erase(stoppedTimes.begin());
		}
		stoppedTimes.push_back(diffMiliseconds);
	}
}

double Stopwatch::lastStoppedTime()
{
	std::lock_guard lock(mut);
	if (stoppedTimes.empty()) {
		return 0.0;
	}
	return stoppedTimes.back();
}

double Stopwatch::stoppedTimeAt(const std::vector<double>::size_type index)
{
	if (index >= stoppedTimes.size()) {
		return lastStoppedTime();
	}

	std::lock_guard lock(mut);
	return stoppedTimes.at(index);
}

std::vector<double> Stopwatch::getStoppedTimes()
{
	std::lock_guard lock(mut);
	auto copy = std::vector<double>();
	std::swap(copy, stoppedTimes);
	return copy;
}

