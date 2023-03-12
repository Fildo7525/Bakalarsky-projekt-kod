#pragma once

#include "controlSW/filter/FrequencyFilter.hpp"

/**
 * @class RobotImpulseFilter
 * @brief Filter used to filter the received motor impulses from the robot.
 *
 * The impulses are check for big fluctuations. Mainly the occasional '0' that sometimes comes up as a speed.
 * This greatly breaks the filter. Thus, we do not use this values.
 */
class RobotImpulseFilter
	: public FrequencyFilter
{
public:
	/**
	 * @brief Constructor
	 *
	 * @see FrequencyFilter Base class of this filter.
	 */
	explicit RobotImpulseFilter(double alpha);

	/**
	 * @brief The same as filter, although the input state is firstly checked for the on/off switch.
	 *
	 * If the input suddenly starts the high filtering would corrupt the output signal. Thus, if the input
	 * suddenly switches from the 'OFF' state (0) the output is reset to the first value. There may happen,
	 * that the input will drop to 0 for one sample. This state is filtered out. Finally because we have
	 * to calculate the output at real time, we cannot alter the older samples. Because of that the output
	 * is set to 'OFF' state (0) one sample after the real off switch is invoked.
	 *
	 * @see filter method to filter the input. It is called internally in this method, too.
	 *
	 * @param input Sample of the input signal.
	 * @return The real output of the filter.
	 */
	double filter(double input) override;

private:
	/// Detects whether the last sample was 0.
	bool m_zeroFlag;
};

