#pragma once

#include <chrono>
#include <vector>
#include <mutex>
#include <type_traits>
using namespace std::chrono_literals;

extern std::mutex mut;

/**
 * @class Stopwatch
 * @brief Benchmark the program using the RAII procedure.
 *
 * Use this class in separate scope with what you want to measure. The class will than save
 * the measured time in a static vector. You can later access the times separately
 * or the vector as a whole "const vector&" using static methods.
 *
 * INFO: We do not need to consern ourselves with linking in multiple modes.
 * The templated classes are not created if they are not used.
 */
template <typename T>
class Stopwatch
{
public:
	/**
	 * @brief Constructor saves a starting timestamp.
	 */
	Stopwatch();

	/**
	 * @brief Destructor calculates the time of its life and saves the time in microseconds (double)
	 * to static vector.
	 */
	~Stopwatch();

	/**
	 * @brief Access the last stopped time in the vector.
	 *
	 * @return Const double reference to the last stopped time.
	 */
	static const double &lastStoppedTime();

	/**
	 * @brief Access a specified stopped time in the vector.
	 *
	 * @param index Index of the timestamp to access.
	 * @return The const double reference to the timestamp.
	 */
	static const double &stoppedTimeAt(const std::vector<double>::size_type index);

	/**
	 * @brief Get the const double reference to all the timestamps.
	 */
	static const std::vector<double> &getStoppedTimes();
private:
	/// Starting timestamp.
	std::chrono::system_clock::time_point m_start;

	static std::vector<double> m_stoppedTimes;
};

extern template
std::vector<double> Stopwatch<std::chrono::milliseconds>::m_stoppedTimes;

template <typename T>
Stopwatch<T>::Stopwatch()
	: m_start(std::chrono::high_resolution_clock::now())
{
	static_assert(std::is_same<std::chrono::seconds, T>::value
				|| std::is_same<std::chrono::milliseconds, T>::value
				|| std::is_same<std::chrono::microseconds, T>::value,
				  "Must be a chrono seconds, milliseconds or microseconds type");
}

template <typename T>
Stopwatch<T>::~Stopwatch()
{
	auto diff = std::chrono::high_resolution_clock::now() - m_start;
	double diffMiliseconds = std::chrono::duration_cast<T>(diff).count();
	{
		std::lock_guard<std::mutex> lock(mut);
		m_stoppedTimes.push_back(diffMiliseconds);
	}
}

template <typename T>
const double &Stopwatch<T>::lastStoppedTime()
{
	{
		std::lock_guard<std::mutex>lock(mut);
		return m_stoppedTimes.back();
	}
}

template <typename T>
const double &Stopwatch<T>::stoppedTimeAt(const std::vector<double>::size_type index)
{
	if (index > m_stoppedTimes.size()) {
		return lastStoppedTime();
	}
	{
		std::lock_guard<std::mutex> lock(mut);
		return m_stoppedTimes.at(index);
	}
}

template <typename T>
const std::vector<double>& Stopwatch<T>::getStoppedTimes()
{
	return m_stoppedTimes;
}

#ifndef NDEBUG

/// Measure the time in micorseconds
#define TICu \
	{ \
		Stopwatch<std::chrono::microseconds> stopwatch;

/// Measure the time in milliseconds
#define TICm \
	{ \
		Stopwatch<std::chrono::milliseconds> stopwatch;

/// Measure the time in seconds
#define TICs \
	{ \
		Stopwatch<std::chrono::seconds> stopwatch;

/// Endpoint of time measuring
#define TOC \
	} \

#else

#define TICu ;
#define TICm ;
#define TICs ;
#define TOC ;

#endif // NDEBUG
