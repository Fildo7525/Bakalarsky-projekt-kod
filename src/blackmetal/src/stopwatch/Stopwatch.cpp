#include "Stopwatch.hpp"

std::mutex mut;
template <T>
std::vector<double> Stopwatch<T>::m_stoppedTimes = std::vector<T>();
