#include "Stopwatch.hpp"
#include <chrono>
#include <mutex>

static std::vector<double> stoppedTimes = std::vector<double>();

std::mutex mut;

Stopwatch::Stopwatch()
	: m_start(std::chrono::high_resolution_clock::now())
{
}

Stopwatch::~Stopwatch()
{
	auto diff = std::chrono::high_resolution_clock::now() - m_start;
	double diffMiliseconds = std::chrono::duration_cast<std::chrono::microseconds>(diff).count();
	{
		std::lock_guard<std::mutex> lock(mut);
		stoppedTimes.push_back(diffMiliseconds);
	}
}

const double &Stopwatch::lastStoppedTime()
{
	std::lock_guard<std::mutex>lock(mut);
	return stoppedTimes.back();
}

const double &Stopwatch::stoppedTimeAt(const std::vector<double>::size_type index)
{
	if (index > stoppedTimes.size()) {
		return lastStoppedTime();
	}
	{
		std::lock_guard<std::mutex> lock(mut);
		return stoppedTimes.at(index);
	}
}

const std::vector<double>& Stopwatch::getStoppedTimes()
{
	return stoppedTimes;
}

