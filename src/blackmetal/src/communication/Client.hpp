#pragma once

#include "ReturnStatus.hpp"

#include <string>
#include <variant>

class Client
{
public:
	Client(int port, const std::string &address);
	virtual ~Client();

	virtual std::variant<bm::Status, std::string> execute(bm::Command cmd, int rightWheel, int leftWheel);
	bm::Status request(int rightWheel, int leftWheel);

	bm::Status send(const std::string &msg);
	bm::Status receive(std::string &msg);

	int socketFD();
protected:
	virtual bm::Status evalReturnState(const std::string &returnJson) = 0;

private:
	/// File descriptor of the object.
	int m_clientFD;
	/// Socket for biding to server, sending and recieving data.
	int m_socket;
};
