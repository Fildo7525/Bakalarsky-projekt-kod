#include "communication/BlackMetal.hpp"
#include "log.hpp"

INIT_MODULE(main)

int main (int argc, char *argv[])
{
	DBG("debug is working");
	INFO("info is working");
	WARN("warning is working");
	ERR("error is working");
	FATAL("fatal is working");
	printf("Hallo world\n");
	return 0;
}

