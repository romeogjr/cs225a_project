#include "SimVizConfigParser.h"

#include <tinyxml2.h>

#include <iostream>

#include "helpers/ConfigParserHelpers.h"
namespace SaiInterfaces {

SimVizConfig SimVizConfigParser::parseConfig(const std::string& config_file) {
	SimVizConfig config;

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(config_file.c_str()) != tinyxml2::XML_SUCCESS) {
		throw std::runtime_error("Could not load simviz config file: " +
								 config_file);
	}

	// Extract the simvizConfiguration element
	tinyxml2::XMLElement* simviz_config_xml =
		doc.FirstChildElement("simvizConfiguration");
	if (!simviz_config_xml) {
		throw std::runtime_error(
			"No 'simvizConfiguration' element found in config file: " +
			config_file);
	}

	// error out if there are multiple simvizConfiguration elements
	if (simviz_config_xml->NextSiblingElement("simvizConfiguration")) {
		throw std::runtime_error(
			"Only one 'simvizConfiguration' element is allowed in config "
			"file: " +
			config_file);
	}

	// Extract the worldFilePath
	if (!simviz_config_xml->Attribute("worldFilePath")) {
		throw std::runtime_error(
			"No 'worldFilePath' Attribute found in simVizConfiguration element "
			"in config file: " +
			config_file);
	}
	config.world_file = simviz_config_xml->Attribute("worldFilePath");

	// Extract the simviz mode
	if (simviz_config_xml->Attribute("mode")) {
		std::string mode_str = simviz_config_xml->Attribute("mode");
		if (mode_str == "simviz") {
			config.mode = SimVizMode::SIMVIZ;
		} else if (mode_str == "simOnly") {
			config.mode = SimVizMode::SIM_ONLY;
		} else if (mode_str == "vizOnly") {
			config.mode = SimVizMode::VIZ_ONLY;
		} else {
			throw std::runtime_error(
				"Invalid simviz mode: " + mode_str + " in config file: " +
				config_file + ". Valid modes are: simviz, simOnly, vizOnly");
		}
	}

	// Extract the flag to publish the mass matrix to redis
	if (simviz_config_xml->Attribute("publishMassMatrixToRedis")) {
		config.publish_mass_matrices_to_redis =
			simviz_config_xml->BoolAttribute("publishMassMatrixToRedis");
	}

	// Extract simParameters
	tinyxml2::XMLElement* simParams =
		simviz_config_xml->FirstChildElement("simParameters");
	if (simParams) {
		if (simParams->NextSiblingElement("simParameters")) {
			throw std::runtime_error(
				"Only one 'simParameters' element is allowed per "
				"'simvizConfiguration'element in config file: " +
				config_file);
		}
		if (simParams->Attribute("timestep")) {
			config.timestep = simParams->DoubleAttribute("timestep");
		}
		if (simParams->Attribute("speedupFactor")) {
			config.speedup_factor = simParams->DoubleAttribute("speedupFactor");
		}
		if (simParams->Attribute("enableJointLimits")) {
			config.enable_joint_limits =
				simParams->BoolAttribute("enableJointLimits");
		}
		if (simParams->Attribute("frictionCoefficient")) {
			config.global_friction_coefficient =
				simParams->DoubleAttribute("frictionCoefficient");
		}
		if (simParams->Attribute("collisionRestitutionCoefficient")) {
			config.global_collision_restitution =
				simParams->DoubleAttribute("collisionRestitutionCoefficient");
		}
		if (simParams->Attribute("enableGravityCompensation")) {
			config.enable_gravity_compensation =
				simParams->BoolAttribute("enableGravityCompensation");
		}
	}

	// Extract model specific dynamic and rendering parameters
	for (tinyxml2::XMLElement* modelParams =
			 simviz_config_xml->FirstChildElement(
				 "robotOrObjectSpecificParameters");
		 modelParams; modelParams = modelParams->NextSiblingElement(
						  "robotOrObjectSpecificParameters")) {
		DynamicAndRenderingParams params;

		if (!modelParams->Attribute("name")) {
			throw std::runtime_error(
				"Robot or object specific parameters must have a name "
				"attribute");
		}
		std::string name = modelParams->Attribute("name");

		if (modelParams->Attribute("dynamicsEnabled")) {
			params.dynamics_enabled =
				modelParams->BoolAttribute("dynamicsEnabled");
		}
		if (modelParams->Attribute("renderingEnabled")) {
			params.rendering_enabled =
				modelParams->BoolAttribute("renderingEnabled");
		}
		if (modelParams->Attribute("jointLimitsEnabled")) {
			params.joint_limits_enabled =
				modelParams->BoolAttribute("jointLimitsEnabled");
		}
		if (modelParams->Attribute("collisionRestitutionCoefficient")) {
			params.collision_restitution_coefficient =
				modelParams->DoubleAttribute("collisionRestitutionCoefficient");
		}
		if (modelParams->Attribute("frictionCoefficient")) {
			params.friction_coefficient =
				modelParams->DoubleAttribute("frictionCoefficient");
		}
		if (modelParams->Attribute("wireMeshRenderingMode")) {
			params.wire_mesh_rendering_mode =
				modelParams->BoolAttribute("wireMeshRenderingMode");
		}
		if (modelParams->Attribute("framesRenderingEnabled")) {
			params.frames_rendering_enabled =
				modelParams->BoolAttribute("framesRenderingEnabled");
		}
		if (modelParams->Attribute("frameSizeWhenRendering")) {
			params.frames_size_when_rendering =
				modelParams->DoubleAttribute("frameSizeWhenRendering");
		}
		config.model_specific_dynamic_and_rendering_params[name] = params;
	}

	// Extract forceSensor elements
	for (tinyxml2::XMLElement* forceSensor =
			 simviz_config_xml->FirstChildElement("forceSensor");
		 forceSensor;
		 forceSensor = forceSensor->NextSiblingElement("forceSensor")) {
		SimForceSensorConfig force_sensor_config;

		if (!forceSensor->Attribute("robotOrObjectName")) {
			throw std::runtime_error(
				"Force sensor must have a 'robotOrObjectName' attribute");
		}
		force_sensor_config.robot_or_object_name =
			forceSensor->Attribute("robotOrObjectName");
		if (forceSensor->Attribute("linkName")) {
			force_sensor_config.link_name = forceSensor->Attribute("linkName");
		}
		if (forceSensor->Attribute("filterCutoff")) {
			force_sensor_config.cutoff_frequency =
				forceSensor->DoubleAttribute("filterCutoff");
		}
		tinyxml2::XMLElement* origin = forceSensor->FirstChildElement("origin");
		if (origin) {
			force_sensor_config.transform_in_link =
				ConfigParserHelpers::parsePose(origin);
		}
		config.force_sensors.push_back(force_sensor_config);
	}

	// extract logger config
	tinyxml2::XMLElement* logger =
		simviz_config_xml->FirstChildElement("logger");
	if (logger) {
		if (logger->NextSiblingElement("logger")) {
			throw std::runtime_error(
				"Only one 'logger' element is allowed per "
				"'simvizConfiguration' "
				"element in config file: " +
				config_file);
		}
		config.logger_config = ConfigParserHelpers::parseLoggerConfig(
			logger, default_logger_folder_name_simviz);
	}

	return config;
}

}  // namespace SaiInterfaces