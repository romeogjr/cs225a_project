#ifndef SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_CONFIG_PARSER_H
#define SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_CONFIG_PARSER_H

#include <tinyxml2.h>

#include "HapticDeviceControllerConfig.h"

namespace SaiInterfaces {

class HapticDeviceControllerConfigParser {
public:
	HapticDeviceControllerConfigParser() = default;
	~HapticDeviceControllerConfigParser() = default;

	std::vector<HapticDeviceControllerConfig> parseConfig(
		const std::string& config_file);

private:
	HapticDeviceControllerConfig parseControllerConfig(
		tinyxml2::XMLElement* controlConfiguration, const int device_number);

	std::string _config_file_name;
	std::vector<HapticDeviceControllerConfig::ControlledRobotTaskConfig> _controlled_robot_tasks;
};

}	// namespace SaiInterfaces

#endif	// SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_CONFIG_PARSER_H