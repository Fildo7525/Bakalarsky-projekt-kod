#include "initializer/BlackMetal.hpp"
#include <cstdlib>
#include <unistd.h>
#include <vector>
#include <iterator>
#include <iostream>

int main(int argc, char const* argv[])
{
	std::vector<long> lws;
	std::vector<long> rws;

	if (argc != 2) {
		std::cout << "Give a speed as argument" << std::endl;
		exit(1);
	}

	double d = std::stod(argv[1]);

	bm::BlackMetal::instance()->execute(bm::Command::SET_LR_WHEEL_VELOCITY, d, -d);

	for (size_t i = 0; i < 20; i++) {
		auto s = bm::BlackMetal::instance()->execute(bm::Command::GET_LR_WHEEL_VELOCITY);
		lws.push_back(s.lws);
		rws.push_back(s.rws);
		usleep(500'000);
	}

	bm::BlackMetal::instance()->execute(bm::Command::SET_LR_WHEEL_VELOCITY, 0, 0);

	std::cout << "LWS:" << std::endl;
	std::copy(lws.cbegin(), lws.cend(), std::ostream_iterator<long>(std::cout, ","));
	std::cout << std::endl;
	std::cout << "RWS:" << std::endl;
	std::copy(rws.cbegin(), rws.cend(), std::ostream_iterator<long>(std::cout, ","));
	std::cout << std::endl;
	return 0;
}

