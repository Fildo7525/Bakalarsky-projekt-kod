#include "RobotDataReceiver.hpp"

#include "ReturnStatus.hpp"
#include "log.hpp"
#include "types/RobotResponseType.hpp"

#include <algorithm>
#include <exception>
#include <iomanip>
#include <thread>
#include <utility>

INIT_MODULE(RobotDataReceiver);

RobotDataReceiver::RobotDataReceiver(int port, const std::string &address)
	: Client(port, address)
	, m_queue(new ts::Queue<RobotRequestType>("m_clientQueue"))
	, m_odometryMessages(new ts::Queue<RobotResponseType>("m_odometryQueue"))
	, m_onVelocityChange([] (RobotResponseType newValue) { (void)newValue;})
{
	std::thread([this] {
		sleep(1);
		workerThread();
	}).detach();
}

bm::Status RobotDataReceiver::sendRequest(bm::Command cmd, RobotRequestType::WheelValueT rightWheel, RobotRequestType::WheelValueT leftWheel)
{
	RobotRequestType message = RobotRequestType().setUserID(1)
									 .setCommand(cmd)
									 .setRighttWheel(rightWheel)
									 .setLeftWheel(leftWheel);

	INFO("sending: " << message.toJson());

	enqueue(message);

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

void RobotDataReceiver::enqueue(const RobotRequestType &msg)
{
	m_queue->push(msg);
}

RobotResponseType RobotDataReceiver::robotVelocity()
{
	DBG("Getting robot velocity");
	auto front = m_odometryMessages->pop();
	return front;
}

void RobotDataReceiver::setOnVelocityChangeCallback(std::function<void(RobotResponseType)> onVelocityChange)
{
	this->m_onVelocityChange = onVelocityChange;
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
			RobotResponseType robotResponse = RobotResponseType::fromJson(response);
			if (m_velocityChangeFlag) {
				DBG("Resetting filter to " << response);
				m_velocityChangeFlag = false;
				m_onVelocityChange(robotResponse);
			}
			m_odometryMessages->push(robotResponse);
		}
	}

	return (responses.size() > 1 ? bm::Status::MULTIPLE_RECEIVE : bm::Status::OK);
}

void RobotDataReceiver::workerThread()
{
	RobotRequestType robotRequest;
	RobotRequestType lastWheelSpeeds;
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

	while (connected()) {
		robotRequest = m_queue->pop();

		if (robotRequest.command() == bm::Command::GET_LR_WHEEL_VELOCITY) {
			getSpeedCommand = true;
		}
		else if (robotRequest.command() == bm::Command::SET_LR_WHEEL_VELOCITY) {
			if (robotRequest.rightWheel() != lastWheelSpeeds.rightWheel() || robotRequest.leftWheel() != lastWheelSpeeds.leftWheel()) {
				DBG("The last value and new value are different. Resetting filters. Last value:\n" << lastWheelSpeeds.toJson() << "\nnew value:\n" << robotRequest.toJson());
				lastWheelSpeeds = robotRequest;
				m_velocityChangeFlag = true;
			}
		}

		std::string message = robotRequest.toJson();
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

