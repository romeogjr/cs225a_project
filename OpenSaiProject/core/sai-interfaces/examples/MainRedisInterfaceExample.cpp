#include "MainRedisInterface.h"

int main(int argc, char** argv) {
	// add world file folder to search path for parser
	SaiModel::URDF_FOLDERS["WORLD_FILES_FOLDER"] =
		std::string(EXAMPLE_FOLDER_PATH) + "/world_files";

	// define the config folder. Only config files in that folder can be used by
	// this application
	std::string config_folder_path =
		std::string(EXAMPLE_FOLDER_PATH) + "/config_files";

	// initial config file (optionnal)
	std::string config_file = "panda_simviz_control.xml";
	if(argc > 1) {
		config_file = argv[1];
	}

	SaiInterfaces::MainRedisInterface main_interface(config_folder_path,
													  config_file);

	return 0;
}