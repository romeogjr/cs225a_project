#ifndef SAI_INTERFACES_ROBOT_CONTROLLER_CONFIG_PARSER_H
#define SAI_INTERFACES_ROBOT_CONTROLLER_CONFIG_PARSER_H

#include <tinyxml2.h>

#include "RobotControllerConfig.h"

namespace SaiInterfaces {

/**
 * @brief Class to parse a robot controller config file and return a
 * RobotControllerConfig object
 *
 */
class RobotControllerConfigParser {
public:
	RobotControllerConfigParser() = default;
	~RobotControllerConfigParser() = default;

	/**
	 * @brief Parses the config file and returns a vector of
	 * RobotControllerConfig objects. It will throw an error if the config
	 * cannot be parsed properly.
	 *
	 * @param config_file full path to the config file
	 * @return a vector of RobotControllerConfig objects, one per robot in the
	 * config file
	 */
	std::vector<RobotControllerConfig> parseConfig(
		const std::string& config_file);

private:
	RobotControllerConfig parseControllersConfig(
		tinyxml2::XMLElement* controlConfiguration);
	RobotSingleControllerConfig parseSingleControllerConfig(
		tinyxml2::XMLElement* xml, const std::string& name);

	JointTaskConfig parseJointTaskConfig(tinyxml2::XMLElement* xml);
	MotionForceTaskConfig parseMotionForceTaskConfig(tinyxml2::XMLElement* xml);

	std::string _config_file_name;
};

}  // namespace SaiInterfaces

#endif	// SAI_INTERFACES_ROBOT_CONTROLLER_CONFIG_PARSER_H