#pragma once

#include <mutex>

/**
 * @class LowPassFilter
 * @brief Class implementing Low Pass (LP) filter.
 *
 * To filter out the high frequency of a signal, you can use this class. To initialize an object
 * You have to supply the alpha value. It represents the percentage of how much should
 * the old output value effect the new output value.
 */
class LowPassFilter
{
public:
	/**
	 * @brief Constructor of the low pass filter.
	 *
	 * The filter setups the initial values. These supers the high frequency
	 * of the supplied signal.
	 *
	 * @param alpha Is a value in range [0;1). Specifies the percentage of how much should
	 * the old value effect the output state.
	 * @param initState State at which should the filter start.
	 */
	explicit LowPassFilter(double alpha, double initState = 0);

	/**
	 * @brief Add a new value to the output of the filter.
	 *
	 * @param input The new sample from the input signal.
	 * @return New output value already counted with the input.
	 */
	virtual double filter(double input);

	/**
	 * @brief Resets the output state of the filter to the supplied value.
	 *
	 * @param newValue State the filter will be after reset.
	 * @return The old state of the filter.
	 */
	double resetInitState(double newValue);

	/**
	 * @brief Returns the state of the output of the filter.
	 */
	double output();

private:
	/// The percentage of how much should the old output effect the new output.
	double m_alpha;

	/// The output of the filter.
	double m_output;

	/// Mutex used for accessing data from multiple threads.
	std::mutex m_mutex;
};
