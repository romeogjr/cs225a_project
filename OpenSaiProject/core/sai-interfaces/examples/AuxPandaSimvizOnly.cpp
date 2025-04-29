#include "simviz/SimVizRedisInterface.h"
#include "simviz/SimVizConfigParser.h"

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["WORLD_FILES_FOLDER"] =
		std::string(EXAMPLE_FOLDER_PATH) + "/world_files";

	std::string config_file =
		std::string(EXAMPLE_FOLDER_PATH) + "/config_files/aa_detailled_panda_simviz_only.xml";

	SaiInterfaces::SimVizConfigParser parser;
	SaiInterfaces::SimVizConfig config = parser.parseConfig(config_file);

	SaiInterfaces::SimVizRedisInterface simviz(config);
	simviz.run();

	return 0;
}