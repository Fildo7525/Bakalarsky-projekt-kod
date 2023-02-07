#include <arpa/inet.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <memory>
#define PORT 665

struct Speed
{
	long lws;
	long rws;
};

namespace bm
{

enum class Command {
	EMPTY,
	EMG_SOPT,
	NORMAL_STOP,
	SET_LR_WHEEL_VELOCITY,
	NONE_4,
	NONE_5,
	GET_LR_WHEEL_VELOCITY,
	PREPARE_WHEEL_CONTROLER,
};

class BlackMetal
{
public:
	enum class Status {
		OK,
		FULL_BUFFER,
		RECIEVE_ERROR,
	};

	static std::shared_ptr<BlackMetal> instance();

	~BlackMetal();
	Speed execute(Command cmd, const double leftWheelVelocity = 0, const double rightWheelVelocity = 0);
private:
	BlackMetal();
	BlackMetal(const BlackMetal &) = delete;
	BlackMetal(BlackMetal &&) = delete;

	Speed parseData(std::string jsonMessage);

	Status evalReturnState(const char *returnJson);

private:
	static bool m_initialized;
	static std::shared_ptr<BlackMetal> m_instance;

	int m_clientFD;
	int m_socket;
};

} // namespace bm

