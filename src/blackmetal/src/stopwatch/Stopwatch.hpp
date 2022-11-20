#pragma once

#include <chrono>
#include <vector>

using namespace std::chrono_literals;

/**
 * @class Stopwatch
 * @brief Benchmark the program using the RAII procedure.
 *
 * Use this class in separate scope with what you want to measure. The class will than save
 * the measured time in a static vector. You can later access the times separately
 * or the vector as a whole "const vector&" using static methods.
 * TODO: The possibility of measuring the time in different units.
 * Default microseconds. This can be done using templates while checking the type in constructor.
 */
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
	 * If the index parameter is bigger than the size of the vector the last stopped time is returned.
	 *
	 * @param index Index of the timestamp to access.
	 * @return The const double reference to the timestamp.
	 */
	static const double &stoppedTimeAt(const std::vector<double>::size_type index);

	/**
	 * @brief Get the const double reference to all the timestamps.
	 */
	[[deprecated("getStoppedTimes is not thread safe. Use stoppedTimeAt instead.")]]
	static const std::vector<double> &getStoppedTimes();
private:
	/// Starting timestamp.
	std::chrono::system_clock::time_point m_start;
};

#define TIC \
	{ \
		Stopwatch stopwatch;

#define TOC \
	} \

