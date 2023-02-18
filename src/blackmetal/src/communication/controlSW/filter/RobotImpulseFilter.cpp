#include "RobotImpulseFilter.hpp"

RobotImpulseFilter::RobotImpulseFilter(double alpha)
	: LowPassFilter(alpha, 0)
	, m_zeroFlag(false)
{
}

double RobotImpulseFilter::filter(double input)
{
	if (input == 0) {
		if (m_zeroFlag) {
			// zeroFlag = false;
			resetInitState(0);
		}
		else {
			m_zeroFlag = true;
			input = output();
		}
	}
	else {
		if (m_zeroFlag) {
			m_zeroFlag = false;
			resetInitState(input);
		}
	}

	return this->LowPassFilter::filter(input);
}

