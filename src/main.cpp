#include "initializer/BlackMetal.hpp"

int main(int argc, char const* argv[])
{
	bm::BlackMetal::instance()->execute(bm::Command::SET_LR_WHEEL_VELOCITY, 1, 1);
	return 0;
}

