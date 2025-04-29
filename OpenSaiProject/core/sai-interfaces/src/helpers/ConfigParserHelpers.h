#ifndef SAI_INTERFACES_CONFIG_PARSER_HELPERS_H
#define SAI_INTERFACES_CONFIG_PARSER_HELPERS_H

#include <tinyxml2.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "CommonConfigs.h"

// \cond
// for internal use only
namespace SaiInterfaces {
namespace ConfigParserHelpers {
Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);
std::vector<std::string> splitString(const std::string& str,
									 const std::vector<char>& separators = {
										 ' ', '\t', '\n', ',', '[', ']'});
Eigen::Vector3d parseVector3d(const char* vec_str);
Eigen::Vector3d parseVector3d(tinyxml2::XMLElement* xml,
							  std::string attribute_name = "xyz");

Eigen::VectorXd parseVectorXd(const char* vec_str);

Eigen::Affine3d parsePose(tinyxml2::XMLElement* xml);

LoggerConfig parseLoggerConfig(tinyxml2::XMLElement* logger,
							   const std::string& default_folder_name);

RedisConfig parseRedisConfig(tinyxml2::XMLElement* redis);

}  // namespace ConfigParserHelpers
}  // namespace SaiInterfaces

// \endcond

#endif	// SAI_INTERFACES_CONFIG_PARSER_HELPERS_H