#include "initializer/BlackMetal.hpp"
#include "initializer/RobotImpulseFilter.hpp"
#include <cstdlib>
#include <unistd.h>
#include <vector>
#include <iterator>
#include <iostream>
#include <chrono>
#include <thread>

void prepareControlers();
void circle(int argc, char const* argv[]);
void line(int argc, char const* argv[]);
void test(int argc, char const* argv[]);

int main(int argc, char const* argv[])
{

	// prepareControlers();
	line(argc, argv);
}

void prepareControlers()
{
	bm::BlackMetal::instance()->execute(bm::Command::PREPARE_WHEEL_CONTROLER);
}

void circle(int argc, char const* argv[])
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
}

void line(int argc, char const* argv[])
{
	std::vector<long> lws;
	std::vector<long> rws;
	RobotImpulseFilter lf(0.8);
	std::vector<long> lws_f;
	RobotImpulseFilter rf(0.8);
	std::vector<long> rws_f;

									// 0.1    -0.1  0.2   -0.2    0.3   -0.3     0.4    -0.4    0.5     -0.5   0.6    -0.6    0.7     -0.7   0.8    -0.8
	std::vector<double> requests = {0.125, -0.125, 0.250, -0.250, 0.375, -0.375, 0.500, -0.500, 0.625, -0.625, 0.750, -0.750, 0.875, -0.875, 1.000, -1.000};

	for(auto speed : requests) {
		bm::BlackMetal::instance()->execute(bm::Command::SET_LR_WHEEL_VELOCITY, speed, speed);

		for (size_t i = 0; i < 20; i++) {
			auto start = std::chrono::high_resolution_clock::now();
			auto s = bm::BlackMetal::instance()->execute(bm::Command::GET_LR_WHEEL_VELOCITY);
			lws.push_back(s.lws);
			rws.push_back(s.rws);

			// Leave out the first sample
			if (i == 0) {
				continue;
			}

			if (i == 1) {
				lf.resetInitState(s.lws);
				rf.resetInitState(s.rws);
			}

			lws_f.push_back(lf.filter(s.lws));
			rws_f.push_back(rf.filter(s.rws));
			auto diff = std::chrono::high_resolution_clock::now() - start;
			std::this_thread::sleep_for(std::chrono::microseconds(100'000) - diff);
		}
		auto s = bm::BlackMetal::instance()->execute(bm::Command::SET_LR_WHEEL_VELOCITY, 0, 0);

		std::cout << "Speed: " << speed << std::endl;
		std::cout << "LWS:" << std::endl;
		std::copy(lws.cbegin(), lws.cend(), std::ostream_iterator<long>(std::cout, ","));
		std::cout << '\n';
		std::copy(lws_f.cbegin(), lws_f.cend(), std::ostream_iterator<long>(std::cout, ","));
		std::cout << std::endl;
		std::cout << "RWS:" << std::endl;
		std::copy(rws.cbegin(), rws.cend(), std::ostream_iterator<long>(std::cout, ","));
		std::cout << '\n';
		std::copy(rws_f.cbegin(), rws_f.cend(), std::ostream_iterator<long>(std::cout, ","));

		std::cout << "\n\n";
		lws.clear();
		rws.clear();
		lws_f.clear();
		rws_f.clear();
		usleep(250'000);
	}

}

void test(int argc, char const* argv[])
{
	double d = std::stod(argv[1]);
	bm::BlackMetal::instance()->execute(bm::Command::SET_LR_WHEEL_VELOCITY, d, d);
	sleep(2);
}

