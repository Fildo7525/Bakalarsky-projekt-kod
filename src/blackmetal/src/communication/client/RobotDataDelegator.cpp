#include "RobotDataDelegator.hpp"

#include "ReturnStatus.hpp"
#include "log.hpp"
#include "types/RobotResponseType.hpp"

#include <algorithm>
#include <exception>
#include <iomanip>
#include <thread>
#include <utility>

INIT_MODULE(RobotDataDelegator);

RobotDataDelegator::RobotDataDelegator(int port, const std::string &address)
	: Client(port, address)
	, m_queue(new ts::Queue<RobotRequestType>("m_clientQueue"))
	, m_odometryMessages(new ts::Queue<RobotResponseType>("m_odometryQueue"))
	, m_onVelocityChange([] (RobotRequestType newValue) { (void)newValue;})
{
	std::thread([this] {
		// We want to create a simultated delay for other components to catch up.
		std::this_thread::sleep_for(1s);
		workerThread();
	}).detach();
}

bm::Status RobotDataDelegator::sendRequest(bm::Command cmd, RobotRequestType::WheelValueT rightWheel, RobotRequestType::WheelValueT leftWheel)
{
	RobotRequestType message = RobotRequestType().setUserID(1)
									 .setCommand(cmd)
									 .setRightWheel(rightWheel)
									 .setLeftWheel(leftWheel);

	INFO("Enqueuing: " << message.toJson());

	enqueue(message);

	return bm::Status::OK;
}

bm::Status RobotDataDelegator::requestSpeed(double rightWheel, double leftWheel)
{
	return sendRequest(bm::Command::SET_LR_WHEEL_VELOCITY, rightWheel, leftWheel);
}

bm::Status RobotDataDelegator::requestPosition(long rightWheel, long leftWheel)
{
	return sendRequest(bm::Command::SET_LR_WHEEL_POSITION, rightWheel, leftWheel);
}

RobotResponseType RobotDataDelegator::robotVelocity()
{
	DBG("Getting robot velocity");
	auto front = m_odometryMessages->pop();
	return front;
}

void RobotDataDelegator::setOnVelocityChangeCallback(std::function<void(RobotRequestType)> onVelocityChange)
{
	this->m_onVelocityChange = onVelocityChange;
}

std::vector<std::string> RobotDataDelegator::splitResponses(const std::string &msg)
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

void RobotDataDelegator::enqueue(const RobotRequestType &msg)
{
	m_queue->push(msg);
	FATAL("Queue size is " << m_queue->size());
}

bm::Status RobotDataDelegator::receive(std::string &msg)
{
	auto status = this->Client::receive(msg);
	if (status != bm::Status::OK) {
		return status;
	}

	auto responses = splitResponses(msg);

	// There may be a situation that the server will send more than one string before we read it.
	// Therefore we will read more strings at once and the odometry will crush.
	DBG("The client received " << msg.size() << " bytes");
	for (auto response : responses) {
		status = evalReturnState(response);
		if (status == bm::Status::ODOMETRY_SPEED_DATA) {
			INFO("Passing " << std::quoted(response) << " to odometry");
			RobotResponseType robotResponse = RobotResponseType::fromJson(response);
			m_odometryMessages->push(robotResponse);
		}
	}

	return (responses.size() > 1 ? bm::Status::MULTIPLE_RECEIVE : bm::Status::OK);
}

void RobotDataDelegator::workerThread()
{
	RobotRequestType robotRequest;
	RobotRequestType lastWheelSpeeds;
	bool getSpeedCommand = false;
	std::string lastSendMsg = "";
	bm::Status lastSendStatus = bm::Status::OK;

	// Lambda function used for receiving messages from the server.
	auto _receive = [this] (std::string &message) -> bm::Status {
		auto receiveStatus = receive(message);
		if (receiveStatus == bm::Status::MULTIPLE_RECEIVE) {
			WARN("Multiple responses read and evaluated at once skipping second read");
		}
		else if (receiveStatus != bm::Status::OK) {
			FATAL("The message could not be received with return state: " << toString(receiveStatus));
		}
		else {
			message = "";
			DBG("The data were received");
		}
		return receiveStatus;
	};

	// Lambda function used for sending messages to the server.
	auto _send = [this, &lastSendMsg, &lastSendStatus] (const std::string &message) -> bm::Status {
		if (lastSendMsg == message && !message.ends_with("\"Command\":6}") && lastSendStatus == bm::Status::OK) {
			return bm::Status::OK;
		}

		SUCCESS("Sending message: " << message);
		auto sendStatus = this->send(message);
		if (sendStatus != bm::Status::OK) {
			FATAL("Could not send: " << message << ".");
		}

		lastSendMsg = message;
		lastSendStatus = sendStatus;

		return sendStatus;
	};

	while (connected()) {
		robotRequest = m_queue->pop();

		if (robotRequest.command() == bm::Command::GET_LR_WHEEL_VELOCITY) {
			getSpeedCommand = true;
		}
		else if (robotRequest.command() == bm::Command::SET_LR_WHEEL_VELOCITY) {
			if (robotRequest.rightWheel() != lastWheelSpeeds.rightWheel() || robotRequest.leftWheel() != lastWheelSpeeds.leftWheel()) {
				DBG("Value change.\n Last value:\n" << lastWheelSpeeds.toJson() << "\nnew value:\n" << robotRequest.toJson());
				lastWheelSpeeds = robotRequest;

				// FROM_IMP_TO_MPS_x is used to convert impulses per second to meters per second using the division.
				// In case we want to convert from meters per second to impulses per second we must use multiplication.
				RobotRequestType request = RobotRequestType()
					.setUserID(1)
					.setCommand(robotRequest.command())
					.setLeftWheel(std::get<double>(robotRequest.leftWheel()) * FROM_IMP_TO_MPS_L)
					.setRightWheel(std::get<double>(robotRequest.rightWheel()) * FROM_IMP_TO_MPS_R);
				m_onVelocityChange(request);
			}
		}

		std::string message = robotRequest.toJson();
		INFO("sending: " << message);

		// Leave this iteration due to the error in communication.
		if (_send(message) != bm::Status::OK) {
			continue;
		}

		// If multiple messages are read at once they are evaluated in reveive function.
		auto readStatus = _receive(message);
		if (readStatus == bm::Status::MULTIPLE_RECEIVE || readStatus != bm::Status::OK) {
			continue;
		}

		if (getSpeedCommand) {
			getSpeedCommand = false;
			std::string wheelSpeed;

			readStatus = _receive(wheelSpeed);
			if (readStatus != bm::Status::OK) {
				continue;
			}

			DBG("Pushing data " << std::quoted(wheelSpeed) << " to m_odometryMessages");
		}
	}
}

