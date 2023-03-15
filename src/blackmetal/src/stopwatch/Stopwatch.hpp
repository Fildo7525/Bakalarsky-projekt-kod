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
	 *
	 * @param maxLength The maximal number of timestamps this instance will allow.
	 */
	Stopwatch(size_t maxLength = 1000);

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
	static double lastStoppedTime();

	/**
	 * @brief Access a specified stopped time in the vector.
	 * If the index parameter is bigger than the size of the vector the last stopped time is returned.
	 *
	 * @param index Index of the timestamp to access.
	 * @return The const double reference to the timestamp.
	 */
	static double stoppedTimeAt(const std::vector<double>::size_type index);

	/**
	 * @brief Get the copy of all the stopped times. The vector capturing them 
	 * is cleared.
	 */
	static std::vector<double> getStoppedTimes();

private:
	/// Starting timestamp.
	std::chrono::system_clock::time_point m_start;

	/// Maximal number of timestamps that this instance will allow.
	size_t m_maxLength;
};

#define TIC \
	{ \
		Stopwatch stopwatch;

#define TOC \
	} \

