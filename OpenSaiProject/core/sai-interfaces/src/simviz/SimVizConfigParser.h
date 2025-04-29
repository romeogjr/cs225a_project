#ifndef SAI_INTERFACES_SIMVIZ_CONFIG_PARSER_H
#define SAI_INTERFACES_SIMVIZ_CONFIG_PARSER_H

#include "SimVizConfig.h"

namespace SaiInterfaces
{

/**
 * @brief Class to parse a simviz config file and return a SimVizConfig object
 *
 */
class SimVizConfigParser
{
public:
    SimVizConfigParser() = default;
    ~SimVizConfigParser() = default;

	/**
	 * @brief Parses the config file and returns a SimVizConfig object.
	 * It will throw an error if the config cannot be parsed properly.
	 *
	 * @param config_file full path to the config file
	 * @return a SimVizConfig object
	 */
    SimVizConfig parseConfig(const std::string& config_file);
};

} // namespace SaiInterfaces

#endif // SAI_INTERFACES_SIMVIZ_CONFIG_PARSER_H