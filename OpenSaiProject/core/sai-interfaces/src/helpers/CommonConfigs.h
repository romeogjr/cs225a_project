#ifndef SAI_INTERFACES_CONFIG_COMMON_CONFIGS_H
#define SAI_INTERFACES_CONFIG_COMMON_CONFIGS_H

#include <string>

namespace SaiInterfaces {
/**
 * @brief Configuration struct for the logger object (common to simviz and
 * controller configs)
 *
 * This is parsed from the xml file from the following element:
 * 	<logger folderName="..." logFrequency="..." enabledAtStartup="..."
 * addTimestampToFilename="..." />
 *
 */
struct LoggerConfig {
	/// @brief The path to the folder where the log files will be saved
	std::string folder_name = "log_files";
	/// @brief The frequency at which the logger will log the data
	double frequency = 100.0;
	/// @brief Whether to start the logger when the simulation starts
	bool start_with_logger_on = false;
	/// @brief Whether to add a timestamp to the filename of the log files
	bool add_timestamp_to_filename = true;

	bool operator==(const LoggerConfig& other) const {
		return (folder_name == other.folder_name) &&
			   (frequency == other.frequency) &&
			   (start_with_logger_on == other.start_with_logger_on) &&
			   (add_timestamp_to_filename == other.add_timestamp_to_filename);
	}

	LoggerConfig(const std::string& folder_name) : folder_name(folder_name) {}
};

/**
 * @brief Configuration struct for the redis config object (one per config file, used by simviz and controllers)
 *
 * This is parsed from the xml file from the following element:
 * 	<redisConfig namespacePrefix="..." ip="..." port="..." />
 *
 */
struct RedisConfig {
	/// @brief The address of the redis server
	std::string redis_ip = "127.0.0.1";

	/// @brief The port of the redis server
	int redis_port = 6379;

	/// @brief The default redis namespace prefix
	std::string redis_namespace_prefix = "sai";
};

}  // namespace SaiInterfaces

#endif	// SAI_INTERFACES_CONFIG_COMMON_CONFIGS_H