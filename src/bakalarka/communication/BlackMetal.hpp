#pragma once

#include <string>
#include <memory>

/**
 * @namespace bm
 * @brief namespace containing the enums for blackmetal constants.
 */
namespace bm
{

/**
 * @enum Command
 * @brief Commands defined in the blackmetal robot documentation.
 */
enum class Command {
	/// Empty command
	EMPTY,
	/// Emergency stop
	EMG_SOPT,
	/// Normal stop
	NORMAL_STOP,
	/// Sets Left and right wheel velocity.
	/// The velocities are required for this command.
	SET_LR_WHEEL_VELOCITY,
	/// Not implemented.
	NONE_4,
	/// Not implemented.
	NONE_5,
	/// Get left and right wheel velocities.
	GET_LR_WHEEL_VELOCITY,
	/// Prepare the controller of left and right wheel.
	PREPARE_WHEEL_CONTROLLER,
};

} // namespace bm

/**
 * @class BlackMetal
 * @brief Singleton class for communication with the blackmetal robot.
 *
 * The class creates client and returns this instance on every call.
 */
class BlackMetal
{
public:
	/**
	 * @enum Status
	 * @brief return status of the server.
	 * The robot receives the json string and returns a json string with a specific return status.
	 * This status is than mapped on BlackMetal::Status enum type.
	 */
	enum class Status {
		/// Server processed the request.
		OK,
		/// Server could not process the request. The buffer is full.
		FULL_BUFFER,
		/// The response could not be recieved.
		RECIEVE_ERROR,
	};

	/**
	 * @brief Return instance of BlackMetal.
	 * The instance is m_initialized only once and continuously reused.
	 */
	static std::shared_ptr<BlackMetal> instance();

	/// Destructor.
	~BlackMetal();

	/**
	 * @brief Send a json string containing the following volues to the robot.
	 *
	 * @param cmd Command to execute.
	 * @param leftWheelVelocity Left wheel velocity. Not mendatory.
	 * @param rightWheelVelocity Right wheel velocity. Not mendatory.
	 * @return The return status of the server.
	 */
	Status execute(bm::Command cmd, const int leftWheelVelocity = 0, const int rightWheelVelocity = 0);
private:
	/// Constructor.
	BlackMetal();
	/// Deleted copy constructor.
	BlackMetal(const BlackMetal &) = delete;
	/// Deleted move constructor.
	BlackMetal(BlackMetal &&) = delete;

	/**
	 * @brief Map the request response json string.
	 *
	 * Mappes the server response json string to thre return status defined in the class.
	 *
	 * @param returnJson Json string.
	 * @return return status code.
	 */
	Status evalReturnState(const char *returnJson);

private:
	/// Checkes if an object was already initialized.
	static bool m_initialized;
	/// Holds the instance of the only object.
	static std::shared_ptr<BlackMetal> m_instance;

	/// File descriptor of the object.
	int m_clientFD;
	/// Socket for biding to server, sending and recieving data.
	int m_socket;
};

