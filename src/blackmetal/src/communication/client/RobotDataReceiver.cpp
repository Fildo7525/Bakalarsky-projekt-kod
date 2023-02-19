#include "RobotDataReceiver.hpp"
#include <algorithm>
#include <iomanip>
#include <log.hpp>
#include <thread>

INIT_MODULE(RobotDataReceiver);

RobotDataReceiver::RobotDataReceiver(int port, const std::string &address)
	: Client(port, address)
	, m_queue(new ts::Queue<std::string>("m_clientQueue"))
	, m_odometryMessages(new ts::Queue<std::string>("m_odometryQueue"))
{
	std::thread([this] {
		sleep(1);
		workerThread();
	}).detach();
}

bm::Status RobotDataReceiver::sendRequest(bm::Command cmd, RobotRequestType::WheelValueT rightWheel, RobotRequestType::WheelValueT leftWheel)
{
	DBG("Composing command: " << bm::stringifyCommand(cmd));
	// Example: "{\"UserID\":1,\"Command\":3,\"RightWheelSpeed\":0.1,\"LeftWheelSpeed\":0.1,\"RightWheelPosition\":0.1,\"LeftWheelPosition\":0.1}";
	std::string message = "{\"UserID\":1,\"Command\":";
	message += std::to_string(int(cmd));
	if (cmd == bm::Command::SET_LR_WHEEL_VELOCITY) {
		message += ",\"RightWheelSpeed\":" + std::to_string(std::get<double>(rightWheel)) +
					",\"LeftWheelSpeed\":" + std::to_string(std::get<double>(leftWheel));
	} else if (cmd == bm::Command::SET_LR_WHEEL_POSITION) {
		message += ",\"RightWheelPosition\":" + std::to_string(std::get<long>(rightWheel)) +
					",\"LeftWheelPosition\":" + std::to_string(std::get<long>(leftWheel));
	}
	message += "}";

	INFO("sending: " << message);

	this->enqueue(message);

	return bm::Status::OK;
}

bm::Status RobotDataReceiver::requestSpeed(double rightWheel, double leftWheel)
{
	return sendRequest(bm::Command::SET_LR_WHEEL_VELOCITY, rightWheel, leftWheel);
}

bm::Status RobotDataReceiver::requestPosition(long rightWheel, long leftWheel)
{
	return sendRequest(bm::Command::SET_LR_WHEEL_POSITION, rightWheel, leftWheel);
}

std::vector<std::string> RobotDataReceiver::splitResponses(const std::string &msg)
{
	INFO("Received message " << msg);
	size_t numberOfReceivedMsgs = std::count_if(msg.cbegin(), msg.cend(), [] (char c) { return c == '}'; });
	std::vector<std::string> receivedStrings(numberOfReceivedMsgs);

	auto start = 0;
	auto endOfJson = msg.find_first_of('}') + 1;
	for (size_t i = 0; i < numberOfReceivedMsgs; i++) {
		receivedStrings[i] = (msg.substr(start, endOfJson));
		DBG("Message received: " << receivedStrings[i]);
		start = endOfJson;
		endOfJson = msg.find_first_of('}', start);
	}

	return receivedStrings;
}

void RobotDataReceiver::enqueue(const std::string &msg)
{
	m_queue->push(msg);
}

std::string RobotDataReceiver::robotVelocity()
{
	DBG("Getting robot velocity");
	auto front = m_odometryMessages->pop();
	return front;
}

bm::Status RobotDataReceiver::receive(std::string &msg)
{
	auto status = this->Client::receive(msg);
	if (status != bm::Status::OK) {
		return status;
	}

	auto responses = splitResponses(msg);

	// There may be a situation that the server will send more than one string before we read it.
	// Therefore we will read more strings at once and the odometry will crush.
	INFO("The client received " << msg.size() << " bytes");
	for (auto response : responses) {
		status = evalReturnState(response);
		if (status == bm::Status::ODOMETRY_SPEED_DATA) {
			INFO("Passing " << std::quoted(response) << " to odometry");
			m_odometryMessages->push(response);
		}
	}

	status = bm::Status::OK;
	if (responses.size() > 1) {
		status = bm::Status::MULTIPLE_RECEIVE;
	}

	return status;
}

void RobotDataReceiver::workerThread()
{
	std::string message;
	bool getSpeedCommand = false;

	// Lambda function used for receiving messages from the server.
	auto _receive = [this] (std::string &message) -> bm::Status {
		auto receiveStatus = receive(message);
		if (receiveStatus == bm::Status::MULTIPLE_RECEIVE) {
			WARN("Multiple responses read and evaluated at once skipping second read");
		}
		else if (receiveStatus != bm::Status::OK) {
			FATAL("The message could not be received with return state: " << stringifyStatus(receiveStatus));
		}
		else {
			message = "";
			DBG("The data were received");
		}
		return receiveStatus;
	};

	// Lambda function used for sending messages to the server.
	auto _send = [this] (const std::string &message) -> bm::Status {
		auto sendStatus = this->send(message);
		if (sendStatus != bm::Status::OK) {
			FATAL("Could not send: " << message << ".");
		}
		else {
			DBG("The data were sent");
		}
		return sendStatus;
	};

	while (m_connected) {
		message = m_queue->pop();

		if (message.find("Command\":6") != std::string::npos) {
			getSpeedCommand = true;
		}
		else if (message.find("Command\":3") != std::string::npos) {
		}

		INFO("sending: " << message);
		_send(message);
		// If multiple messages are read at once they are evaluated in reveive function.
		if (_receive(message) == bm::Status::MULTIPLE_RECEIVE) {
			continue;
		}

		if (getSpeedCommand) {
			getSpeedCommand = false;
			std::string wheelSpeed;
			// We take a risk and do not check for an error. The connection is established at this point.
			// May be changed in the future.
			_receive(wheelSpeed);
			INFO("Pushing data " << wheelSpeed << " to m_odometryMessages");
		}
	}
}

