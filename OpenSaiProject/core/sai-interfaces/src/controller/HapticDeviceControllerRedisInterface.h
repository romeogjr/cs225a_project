#ifndef SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_REDIS_INTERFACE_H
#define SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_REDIS_INTERFACE_H

#include <mutex>
#include <string>

#include "HapticDeviceControllerConfig.h"
#include "SaiCommon.h"
#include "SaiPrimitives.h"

namespace SaiInterfaces {

/**
 * @brief Class to run a haptic device controller from a custom xml config file
 * and provide an easy method of interaction via redis
 *
 */
class HapticDeviceControllerRedisInterface {
public:
	HapticDeviceControllerRedisInterface(
		const HapticDeviceControllerConfig& config,
		const bool setup_signal_handler = true);
	~HapticDeviceControllerRedisInterface() = default;

	/**
	 * @brief Run the controller. This function will run until the
	 * user_stop_signal is set to true externally, or the signal handler is
	 * triggered bu ctrl+c is it was setup.
	 *
	 * @param user_stop_signal
	 */
	void run(const std::atomic<bool>& user_stop_signal = false);

private:
	enum LoggingState {
		OFF,
		START,
		ON,
		STOP,
	};

	void initialize();
	void runRedisCommunication(const std::atomic<bool>& user_stop_signal);
	void processInputs();
	void initialParametrizationFromConfig();


	HapticDeviceControllerConfig _config;

	std::unique_ptr<SaiPrimitives::HapticDeviceController>
		_haptic_controller;

	std::unique_ptr<SaiCommon::RedisClient> _redis_client;

	SaiPrimitives::HapticControllerInput _haptic_input;
	SaiPrimitives::HapticControllerOutput _haptic_output;
	bool _haptic_device_switch_pressed;
	bool _haptic_device_switch_was_pressed;

	std::mutex _haptic_output_mutex;

	std::unique_ptr<SaiCommon::Logger> _logger;
	bool _logging_on;
	LoggingState _logging_state;
};

}  // namespace SaiInterfaces

#endif	// SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_REDIS_INTERFACE_H