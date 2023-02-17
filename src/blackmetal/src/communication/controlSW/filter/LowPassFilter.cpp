#include "LowPassFilter.hpp"

LowPassFilter::LowPassFilter(double alpha, double initState)
	: m_alpha(alpha)
	, m_output(initState)
{
}

double LowPassFilter::filter(double input)
{
	std::scoped_lock<std::mutex> lk(m_mutex);
	m_output = m_alpha * m_output + (1.0 - m_alpha) * input;
	return m_output;
}

double LowPassFilter::resetInitState(double newValue)
{
	std::scoped_lock<std::mutex> lk(m_mutex);
	auto cpy = m_output;
	m_output = newValue;
	return cpy;
}

double LowPassFilter::output()
{
	std::scoped_lock<std::mutex> lk(m_mutex);
	return m_output;
}

