#include "RobotControllerConfigParser.h"

#include <iostream>

#include "helpers/ConfigParserHelpers.h"

using namespace std;
using namespace Eigen;

namespace SaiInterfaces {

namespace {

std::string parseInterfaceAttribute(const char* attribute) {
	vector<string> vectorStr = ConfigParserHelpers::splitString(attribute);
	if (vectorStr.size() == 1) {
		return vectorStr[0];
	} else if (vectorStr.size() == 3) {
		return "[" + vectorStr[0] + "," + vectorStr[1] + "," + vectorStr[2] +
			   "]";
	} else {
		throw runtime_error(
			"xml parsing error. Cannot parse scalar or Vector3d from the "
			"string " +
			string(attribute) + " because it should have 1 or 3 values.");
	}
}

enum GainsType {
	JOINT_GAINS,
	MOTFORCE_POS,
	MOTFORCE_ORI,
	MOTFORCE_FORCE,
	MOTFORCE_MOMENT
};

MotionForceTaskConfig::InterfaceConfig parseInterfaceConfig(
	tinyxml2::XMLElement* interface_xml) {
	MotionForceTaskConfig::InterfaceConfig interface_config;

	if (interface_xml->Attribute("minGoalPosition")) {
		interface_config.min_goal_position = parseInterfaceAttribute(
			interface_xml->Attribute("minGoalPosition"));
	}
	if (interface_xml->Attribute("maxGoalPosition")) {
		interface_config.max_goal_position = parseInterfaceAttribute(
			interface_xml->Attribute("maxGoalPosition"));
	}
	if (interface_xml->Attribute("minDesiredForce")) {
		interface_config.min_desired_force = parseInterfaceAttribute(
			interface_xml->Attribute("minDesiredForce"));
	}
	if (interface_xml->Attribute("maxDesiredForce")) {
		interface_config.max_desired_force = parseInterfaceAttribute(
			interface_xml->Attribute("maxDesiredForce"));
	}
	if (interface_xml->Attribute("minDesiredMoment")) {
		interface_config.min_desired_moment = parseInterfaceAttribute(
			interface_xml->Attribute("minDesiredMoment"));
	}
	if (interface_xml->Attribute("maxDesiredMoment")) {
		interface_config.max_desired_moment = parseInterfaceAttribute(
			interface_xml->Attribute("maxDesiredMoment"));
	}
	return interface_config;
}

GainsConfig parseGainsConfig(tinyxml2::XMLElement* xml,
							 const string& config_file_name,
							 const GainsType gains_type) {
	string gains_name;
	double default_kp, default_kv, default_ki;

	switch (gains_type) {
		case JOINT_GAINS:
			gains_name = "jointGains";
			default_kp = JointTaskDefaultParams::kp;
			default_kv = JointTaskDefaultParams::kv;
			default_ki = JointTaskDefaultParams::ki;
			break;
		case MOTFORCE_POS:
			gains_name = "positionGains";
			default_kp = MotionForceTaskDefaultParams::kp_pos;
			default_kv = MotionForceTaskDefaultParams::kv_pos;
			default_ki = MotionForceTaskDefaultParams::ki_pos;
			break;
		case MOTFORCE_ORI:
			gains_name = "orientationGains";
			default_kp = MotionForceTaskDefaultParams::kp_ori;
			default_kv = MotionForceTaskDefaultParams::kv_ori;
			default_ki = MotionForceTaskDefaultParams::ki_ori;
			break;
		case MOTFORCE_FORCE:
			gains_name = "forceGains";
			default_kp = MotionForceTaskDefaultParams::kp_force;
			default_kv = MotionForceTaskDefaultParams::kv_force;
			default_ki = MotionForceTaskDefaultParams::ki_force;
			break;
		case MOTFORCE_MOMENT:
			gains_name = "momentGains";
			default_kp = MotionForceTaskDefaultParams::kp_moment;
			default_kv = MotionForceTaskDefaultParams::kv_moment;
			default_ki = MotionForceTaskDefaultParams::ki_moment;
			break;
	}

	GainsConfig gains_config = GainsConfig(default_kp, default_kv, default_ki);

	const char* kp = xml->Attribute("kp");
	const char* kv = xml->Attribute("kv");
	const char* ki = xml->Attribute("ki");

	vector<string> vectorKp = {};
	vector<string> vectorKv = {};
	vector<string> vectorKi = {};

	if (kp) {
		vectorKp = ConfigParserHelpers::splitString(kp);
	}
	if (kv) {
		vectorKv = ConfigParserHelpers::splitString(kv);
	}
	if (ki) {
		vectorKi = ConfigParserHelpers::splitString(ki);
	}

	const int size =
		max(vectorKp.size(), max(vectorKv.size(), vectorKi.size()));

	if (size == 0) {
		return gains_config;
	}

	if (size == 1) {
		if (vectorKp.size() == 1) {
			gains_config.kp.setConstant(1, stod(vectorKp[0]));
		}
		if (vectorKv.size() == 1) {
			gains_config.kv.setConstant(1, stod(vectorKv[0]));
		}
		if (vectorKi.size() == 1) {
			gains_config.ki.setConstant(1, stod(vectorKi[0]));
		}
		return gains_config;
	}

	if (vectorKp.size() > 1 && vectorKp.size() != size ||
		vectorKv.size() > 1 && vectorKv.size() != size ||
		vectorKi.size() > 1 && vectorKi.size() != size) {
		throw runtime_error(
			gains_name +
			" gains must have the same number of kp, kv and ki values for "
			"those in vector form in config file: " +
			config_file_name);
	}

	if (gains_type == MOTFORCE_FORCE || gains_type == MOTFORCE_MOMENT) {
		throw runtime_error(gains_name +
							" gains must have 1 kp, kv or ki value if present "
							"in config file: " +
							config_file_name);
	}

	if ((gains_type == MOTFORCE_POS || gains_type == MOTFORCE_ORI) &&
		size != 3) {
		throw runtime_error(gains_name +
							" gains must have 3 kp, kv or ki values for "
							"those in vector form in config file: " +
							config_file_name);
	}

	if (vectorKp.size() == 0) {
		gains_config.kp.setConstant(size, default_kp);
	} else if (vectorKp.size() == 1) {
		gains_config.kp.setConstant(size, stod(vectorKp[0]));
	} else {
		VectorXd kp_vec = VectorXd(size);
		for (int i = 0; i < size; i++) {
			kp_vec[i] = stod(vectorKp[i]);
		}
		gains_config.kp = kp_vec;
	}

	if (vectorKv.size() == 0) {
		gains_config.kv.setConstant(size, default_kv);
	} else if (vectorKv.size() == 1) {
		gains_config.kv.setConstant(size, stod(vectorKv[0]));
	} else {
		VectorXd kv_vec = VectorXd(size);
		for (int i = 0; i < size; i++) {
			kv_vec[i] = stod(vectorKv[i]);
		}
		gains_config.kv = kv_vec;
	}

	if (vectorKi.size() == 0) {
		gains_config.ki.setConstant(size, default_ki);
	} else if (vectorKi.size() == 1) {
		gains_config.ki.setConstant(size, stod(vectorKi[0]));
	} else {
		VectorXd ki_vec = VectorXd(size);
		for (int i = 0; i < size; i++) {
			ki_vec[i] = stod(vectorKi[i]);
		}
		gains_config.ki = ki_vec;
	}

	return gains_config;
}

JointTaskConfig::JointOTGConfig parseOTGJointConfig(
	tinyxml2::XMLElement* otg_xml, const string& config_file_name) {
	JointTaskConfig::JointOTGConfig otg_config;

	// type
	const char* type = otg_xml->Attribute("type");
	if (!type) {
		throw runtime_error("otg must have a type in config file: " +
							config_file_name);
	}
	if (string(type) == "disabled") {
		otg_config.enabled = false;
	} else if (string(type) == "acceleration") {
		otg_config.enabled = true;
		otg_config.jerk_limited = false;
	} else if (string(type) == "jerk") {
		otg_config.enabled = true;
		otg_config.jerk_limited = true;
	} else {
		throw runtime_error("Unknown otg type: " + string(type));
	}

	const char* velocity_limit = otg_xml->Attribute("maxVelocity");
	const char* acceleration_limit = otg_xml->Attribute("maxAcceleration");
	const char* jerk_limit = otg_xml->Attribute("maxJerk");

	vector<string> vectorVelocityLimit = {};
	vector<string> vectorAccelerationLimit = {};
	vector<string> vectorJerkLimit = {};

	if (velocity_limit) {
		vectorVelocityLimit = ConfigParserHelpers::splitString(velocity_limit);
	}
	if (acceleration_limit) {
		vectorAccelerationLimit =
			ConfigParserHelpers::splitString(acceleration_limit);
	}
	if (jerk_limit) {
		vectorJerkLimit = ConfigParserHelpers::splitString(jerk_limit);
	}

	const int size =
		max(vectorVelocityLimit.size(),
			max(vectorAccelerationLimit.size(), vectorJerkLimit.size()));

	if (size == 0) {
		return otg_config;
	}

	if (size == 1) {
		if (vectorVelocityLimit.size() == 1) {
			otg_config.limits.max_velocity.setConstant(
				1, stod(vectorVelocityLimit[0]));
		}
		if (vectorAccelerationLimit.size() == 1) {
			otg_config.limits.max_acceleration.setConstant(
				1, stod(vectorAccelerationLimit[0]));
		}
		if (vectorJerkLimit.size() == 1) {
			otg_config.limits.max_jerk.setConstant(1, stod(vectorJerkLimit[0]));
		}
		return otg_config;
	}

	if (vectorVelocityLimit.size() > 1 && vectorVelocityLimit.size() != size ||
		vectorAccelerationLimit.size() > 1 &&
			vectorAccelerationLimit.size() != size ||
		vectorJerkLimit.size() > 1 && vectorJerkLimit.size() != size) {
		throw runtime_error(
			"otg config limits must have the same number of maxVelocity, "
			"maxAcceleration, and maxJerk values for those in vector form in "
			"config file: " +
			config_file_name);
	}

	if (vectorVelocityLimit.size() == 0) {
		otg_config.limits.max_velocity.setConstant(
			size, JointTaskDefaultParams::otg_max_velocity);
	} else if (vectorVelocityLimit.size() == 1) {
		otg_config.limits.max_velocity.setConstant(
			size, stod(vectorVelocityLimit[0]));
	} else {
		VectorXd vel_limits_vec = VectorXd(size);
		for (int i = 0; i < size; i++) {
			vel_limits_vec[i] = stod(vectorVelocityLimit[i]);
		}
		otg_config.limits.max_velocity = vel_limits_vec;
	}

	if (vectorAccelerationLimit.size() == 0) {
		otg_config.limits.max_acceleration.setConstant(
			size, JointTaskDefaultParams::otg_max_acceleration);
	} else if (vectorAccelerationLimit.size() == 1) {
		otg_config.limits.max_acceleration.setConstant(
			size, stod(vectorAccelerationLimit[0]));
	} else {
		VectorXd acc_limits_vec = VectorXd(size);
		for (int i = 0; i < size; i++) {
			acc_limits_vec[i] = stod(vectorAccelerationLimit[i]);
		}
		otg_config.limits.max_acceleration = acc_limits_vec;
	}

	if (vectorJerkLimit.size() == 0) {
		otg_config.limits.max_jerk.setConstant(
			size, JointTaskDefaultParams::otg_max_jerk);
	} else if (vectorJerkLimit.size() == 1) {
		otg_config.limits.max_jerk.setConstant(size, stod(vectorJerkLimit[0]));
	} else {
		VectorXd jerk_limits_vec = VectorXd(size);
		for (int i = 0; i < size; i++) {
			jerk_limits_vec[i] = stod(vectorJerkLimit[i]);
		}
		otg_config.limits.max_jerk = jerk_limits_vec;
	}

	return otg_config;
}

JointTaskConfig::JointVelSatConfig parseVelSatJointConfig(
	tinyxml2::XMLElement* vel_sat_xml, const string& config_file_name) {
	JointTaskConfig::JointVelSatConfig vel_sat_config;

	// enabled
	if (!vel_sat_xml->Attribute("enabled")) {
		throw runtime_error(
			"velocitySaturation must have an enabled attribute if present in "
			"joint task in config file: " +
			config_file_name);
	}
	vel_sat_config.enabled = vel_sat_xml->BoolAttribute("enabled");

	const char* velocity_limits = vel_sat_xml->Attribute("velocityLimit");
	if (velocity_limits) {
		vector<string> vectorVelocityLimit =
			ConfigParserHelpers::splitString(velocity_limits);
		const int size = vectorVelocityLimit.size();
		VectorXd vel_limits_vec = VectorXd(size);
		for (int i = 0; i < size; i++) {
			vel_limits_vec[i] = stod(vectorVelocityLimit[i]);
		}
		vel_sat_config.velocity_limit = vel_limits_vec;
	}
	return vel_sat_config;
}

}  // namespace

std::vector<RobotControllerConfig> RobotControllerConfigParser::parseConfig(
	const std::string& config_file) {
	_config_file_name = config_file;
	std::vector<RobotControllerConfig> configs;

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(config_file.c_str()) != tinyxml2::XML_SUCCESS) {
		throw runtime_error("Could not load controller config file: " +
							config_file);
	}

	if (doc.FirstChildElement("robotControlConfiguration") == nullptr) {
		throw runtime_error(
			"No 'robotControlConfiguration' element found in config file: " +
			config_file);
	}

	// loop over the robotControlConfiguration elements
	for (tinyxml2::XMLElement* robotControlConfiguration =
			 doc.FirstChildElement("robotControlConfiguration");
		 robotControlConfiguration;
		 robotControlConfiguration =
			 robotControlConfiguration->NextSiblingElement(
				 "robotControlConfiguration")) {
		// get the robot name
		if (!robotControlConfiguration->Attribute("robotName")) {
			throw runtime_error(
				"Some of the robotControlConfiguration are missing a "
				"'robotName' attribute field in config file: " +
				config_file);
		}
		const std::string robot_name =
			robotControlConfiguration->Attribute("robotName");

		// get the robot model file
		if (!robotControlConfiguration->Attribute("robotModelFile")) {
			throw runtime_error(
				"Some of the robotControlConfiguration are missing a "
				"'robotModelFile' attribute field in config file: " +
				config_file);
		}
		const std::string robot_model_file =
			robotControlConfiguration->Attribute("robotModelFile");

		configs.push_back(parseControllersConfig(robotControlConfiguration));
		configs.back().robot_name = robot_name;
		configs.back().robot_model_file = robot_model_file;

		// get the controller frequency
		if (robotControlConfiguration->Attribute("controlFrequency")) {
			configs.back().control_frequency =
				robotControlConfiguration->DoubleAttribute("controlFrequency");
		}

		// get whether to get the mass matrix from redis
		if (robotControlConfiguration->Attribute("getMassMatrixFromRedis")) {
			configs.back().get_mass_matrix_from_redis =
				robotControlConfiguration->BoolAttribute(
					"getMassMatrixFromRedis");
		}
	}

	return configs;
}

RobotControllerConfig RobotControllerConfigParser::parseControllersConfig(
	tinyxml2::XMLElement* controlConfiguration) {
	RobotControllerConfig config;

	// robot base in world
	tinyxml2::XMLElement* baseFrame =
		controlConfiguration->FirstChildElement("baseFrame");
	if (baseFrame) {
		config.robot_base_in_world = ConfigParserHelpers::parsePose(baseFrame);
	}

	// world gravity
	tinyxml2::XMLElement* worldGravity =
		controlConfiguration->FirstChildElement("worldGravity");
	if (worldGravity) {
		config.world_gravity = ConfigParserHelpers::parseVector3d(worldGravity);
	}

	// extract logger config
	tinyxml2::XMLElement* logger =
		controlConfiguration->FirstChildElement("logger");
	if (logger) {
		if (logger->NextSiblingElement("logger")) {
			throw runtime_error(
				"Only one logger element is allowed per "
				"'RobotControlConfiguration' in config file: " +
				_config_file_name);
		}
		config.logger_config = ConfigParserHelpers::parseLoggerConfig(
			logger, default_logger_folder_name_robot_controller);
	}

	// parse all controller configs
	for (tinyxml2::XMLElement* controller =
			 controlConfiguration->FirstChildElement("controller");
		 controller;
		 controller = controller->NextSiblingElement("controller")) {
		// get controller name
		const char* name = controller->Attribute("name");
		if (!name) {
			throw runtime_error(
				"controllers must have a name in config file: " +
				_config_file_name);
		}
		if (name == string("")) {
			throw runtime_error(
				"controllers must have a non-empty name in config file: " +
				_config_file_name);
		}
		if (config.single_controller_configs.find(name) !=
			config.single_controller_configs.end()) {
			throw runtime_error(
				"controllers must have a unique name in config file: " +
				_config_file_name);
		}

		config.single_controller_configs[name] =
			parseSingleControllerConfig(controller, name);

		if (config.initial_active_controller_name == "") {
			config.initial_active_controller_name = name;
		}
	}
	return config;
}

RobotSingleControllerConfig
RobotControllerConfigParser::parseSingleControllerConfig(
	tinyxml2::XMLElement* xml, const std::string& name) {
	RobotSingleControllerConfig single_controller_config;
	single_controller_config.controller_name = name;

	if (xml->Attribute("gravityCompensation")) {
		single_controller_config.enable_gravity_compensation =
			xml->BoolAttribute("gravityCompensation");
	}
	if (xml->Attribute("jointLimitAvoidance")) {
		single_controller_config.enable_joint_limit_avoidance =
			xml->BoolAttribute("jointLimitAvoidance");
	}
	if (xml->Attribute("torqueSaturation")) {
		single_controller_config.enable_torque_saturation =
			xml->BoolAttribute("torqueSaturation");
	}

	vector<variant<JointTaskConfig, MotionForceTaskConfig>> task_configs;
	vector<string> controller_task_names;

	// loop over tasks
	for (tinyxml2::XMLElement* element = xml->FirstChildElement(); element;
		 element = element->NextSiblingElement()) {
		const char* elementName = element->Name();
		std::string task_name = "";
		if (strcmp(elementName, "jointTask") == 0) {
			task_configs.push_back(parseJointTaskConfig(element));
			task_name = get<JointTaskConfig>(task_configs.back()).task_name;
		} else if (strcmp(elementName, "motionForceTask") == 0) {
			task_configs.push_back(parseMotionForceTaskConfig(element));
			task_name =
				get<MotionForceTaskConfig>(task_configs.back()).task_name;
		} else {
			throw runtime_error(
				"Unknown task type: " + std::string(elementName) +
				" in config file: " + _config_file_name +
				". Only supported tasks in 'controller' element are "
				"'jointTask' and 'motionForceTask'");
		}
		if (find(controller_task_names.begin(), controller_task_names.end(),
				 task_name) != controller_task_names.end()) {
			throw runtime_error(
				"tasks from the same controller must have a unique name in "
				"config file: " +
				_config_file_name);
		}
		controller_task_names.push_back(task_name);
	}
	single_controller_config.tasks_configs = task_configs;
	return single_controller_config;
}

JointTaskConfig RobotControllerConfigParser::parseJointTaskConfig(
	tinyxml2::XMLElement* xml) {
	JointTaskConfig config;
	// get name
	const char* name = xml->Attribute("name");
	if (!name) {
		throw runtime_error("tasks must have a name in config file: " +
							_config_file_name);
	}
	if (string(name) == "") {
		throw runtime_error(
			"tasks must have a non-empty name in config file: " +
			_config_file_name);
	}
	config.task_name = name;

	// dynamic decoupling
	if (xml->Attribute("useDynamicDecoupling")) {
		config.use_dynamic_decoupling =
			xml->BoolAttribute("useDynamicDecoupling");
	}

	// bie threshold
	if (xml->Attribute("bieThreshold")) {
		config.bie_threshold = xml->DoubleAttribute("bieThreshold");
	}

	// get controlled joints
	tinyxml2::XMLElement* controlled_joints =
		xml->FirstChildElement("controlledJointNames");
	if (controlled_joints && controlled_joints->GetText()) {
		config.controlled_joint_names =
			ConfigParserHelpers::splitString(controlled_joints->GetText());
	}

	// velocity saturation
	tinyxml2::XMLElement* velocity_saturation =
		xml->FirstChildElement("velocitySaturation");
	if (velocity_saturation) {
		config.velocity_saturation_config =
			parseVelSatJointConfig(velocity_saturation, _config_file_name);
	}

	// otg
	tinyxml2::XMLElement* otg = xml->FirstChildElement("otg");
	if (otg) {
		config.otg_config = parseOTGJointConfig(otg, _config_file_name);
	}

	// gains
	tinyxml2::XMLElement* gains = xml->FirstChildElement("gains");
	if (gains) {
		config.gains_config =
			parseGainsConfig(gains, _config_file_name, JOINT_GAINS);
	}

	return config;
}

MotionForceTaskConfig RobotControllerConfigParser::parseMotionForceTaskConfig(
	tinyxml2::XMLElement* xml) {
	MotionForceTaskConfig config;
	// get name
	const char* name = xml->Attribute("name");
	if (!name) {
		throw runtime_error("tasks must have a name in config file: " +
							_config_file_name);
	}
	if (string(name) == "") {
		throw runtime_error(
			"tasks must have a non-empty name in config file: " +
			_config_file_name);
	}
	config.task_name = name;

	// link name
	if (!xml->Attribute("linkName")) {
		throw runtime_error(
			"'motionForceTask' element must have a linkName attribute in "
			"config file: " +
			_config_file_name);
	}
	config.link_name = xml->Attribute("linkName");

	// parametrization in compliant frame
	if (xml->Attribute("parametrizationInCompliantFrame")) {
		config.is_parametrization_in_compliant_frame =
			xml->BoolAttribute("parametrizationInCompliantFrame");
	}

	// use dynamic decoupling
	if (xml->Attribute("useDynamicDecoupling")) {
		config.use_dynamic_decoupling =
			xml->BoolAttribute("useDynamicDecoupling");
	}

	// bie threshold
	if (xml->Attribute("bieThreshold")) {
		config.bie_threshold = xml->DoubleAttribute("bieThreshold");
	}

	// compliant frame
	tinyxml2::XMLElement* compliant_frame =
		xml->FirstChildElement("compliantFrame");
	if (compliant_frame) {
		config.compliant_frame =
			ConfigParserHelpers::parsePose(compliant_frame);
	}

	// controlled directions position
	tinyxml2::XMLElement* controlled_directions_position =
		xml->FirstChildElement("controlledDirectionsTranslation");
	if (controlled_directions_position) {
		vector<Vector3d> controlled_directions;
		for (tinyxml2::XMLElement* direction =
				 controlled_directions_position->FirstChildElement("direction");
			 direction;
			 direction = direction->NextSiblingElement("direction")) {
			controlled_directions.push_back(
				ConfigParserHelpers::parseVector3d(direction));
		}
		config.controlled_directions_position = controlled_directions;
	}

	// controlled directions orientation
	tinyxml2::XMLElement* controlled_directions_orientation =
		xml->FirstChildElement("controlledDirectionsRotation");
	if (controlled_directions_orientation) {
		vector<Vector3d> controlled_directions;
		for (tinyxml2::XMLElement* direction =
				 controlled_directions_orientation->FirstChildElement(
					 "direction");
			 direction;
			 direction = direction->NextSiblingElement("direction")) {
			controlled_directions.push_back(
				ConfigParserHelpers::parseVector3d(direction));
		}
		config.controlled_directions_orientation = controlled_directions;
	}

	// Force control
	tinyxml2::XMLElement* force_control =
		xml->FirstChildElement("forceControl");
	if (force_control) {
		// closed looop force control
		if (force_control->Attribute("closedLoopForceControl")) {
			config.force_control_config.closed_loop_force_control =
				force_control->BoolAttribute("closedLoopForceControl");
		}

		// force sensor frame
		tinyxml2::XMLElement* force_sensor_frame =
			force_control->FirstChildElement("forceSensorFrame");
		if (force_sensor_frame) {
			config.force_control_config.force_sensor_frame =
				ConfigParserHelpers::parsePose(force_sensor_frame);
		}

		// force space param
		tinyxml2::XMLElement* force_space_param =
			force_control->FirstChildElement("forceSpaceParametrization");
		if (force_space_param) {
			MotionForceTaskConfig::ForceMotionSpaceParamConfig
				force_motion_space_config;
			if (force_space_param->Attribute("dim")) {
				force_motion_space_config.force_space_dimension =
					force_space_param->IntAttribute("dim");
			}
			if (force_space_param->Attribute("direction")) {
				force_motion_space_config.axis =
					ConfigParserHelpers::parseVector3d(
						force_space_param->Attribute("direction"));
				if (force_motion_space_config.axis.norm() > 1e-3) {
					force_motion_space_config.axis.normalize();
				}
			}
			config.force_control_config.force_space_param_config =
				force_motion_space_config;
		}

		// moment space param
		tinyxml2::XMLElement* moment_space_param =
			force_control->FirstChildElement("momentSpaceParametrization");
		if (moment_space_param) {
			MotionForceTaskConfig::ForceMotionSpaceParamConfig
				moment_space_param_config;
			if (moment_space_param->Attribute("dim")) {
				moment_space_param_config.force_space_dimension =
					moment_space_param->IntAttribute("dim");
			}
			if (moment_space_param->Attribute("direction")) {
				moment_space_param_config.axis =
					ConfigParserHelpers::parseVector3d(
						moment_space_param->Attribute("direction"));
				if (moment_space_param_config.axis.norm() > 1e-3) {
					moment_space_param_config.axis.normalize();
				}
			}
			config.force_control_config.moment_space_param_config =
				moment_space_param_config;
		}

		// force gains
		tinyxml2::XMLElement* force_gains =
			force_control->FirstChildElement("forceGains");
		if (force_gains) {
			config.force_control_config.force_gains_config = parseGainsConfig(
				force_gains, _config_file_name, MOTFORCE_FORCE);
		}

		// moment gains
		tinyxml2::XMLElement* moment_gains =
			force_control->FirstChildElement("momentGains");
		if (moment_gains) {
			config.force_control_config.moment_gains_config = parseGainsConfig(
				moment_gains, _config_file_name, MOTFORCE_MOMENT);
		}
	}

	// velocity saturation
	tinyxml2::XMLElement* velocity_saturation =
		xml->FirstChildElement("velocitySaturation");
	if (velocity_saturation) {
		MotionForceTaskConfig::VelSatConfig vel_sat_config;
		// enabled
		if (!velocity_saturation->Attribute("enabled")) {
			throw runtime_error(
				"velocitySaturation must have an enabled attribute if present "
				"in MotionForceTask in config file: " +
				_config_file_name);
		}
		vel_sat_config.enabled = velocity_saturation->BoolAttribute("enabled");

		const char* linear_velocity_limit =
			velocity_saturation->Attribute("linearVelocityLimit");
		if (linear_velocity_limit) {
			vel_sat_config.linear_velocity_limit = stod(linear_velocity_limit);
		}

		const char* angular_velocity_limit =
			velocity_saturation->Attribute("angularVelocityLimit");
		if (angular_velocity_limit) {
			vel_sat_config.angular_velocity_limit =
				stod(angular_velocity_limit);
		}
		config.velocity_saturation_config = vel_sat_config;
	}

	// otg
	tinyxml2::XMLElement* otg = xml->FirstChildElement("otg");
	if (otg) {
		MotionForceTaskConfig::OTGConfig otg_config;
		// type
		const char* type = otg->Attribute("type");
		if (!type) {
			throw runtime_error(
				"otg must have a type attribute if present in config file: " +
				_config_file_name);
		}
		if (string(type) == "disabled") {
			otg_config.enabled = false;
		} else if (string(type) == "acceleration") {
			otg_config.enabled = true;
			otg_config.jerk_limited = false;
		} else if (string(type) == "jerk") {
			otg_config.enabled = true;
			otg_config.jerk_limited = true;
		} else {
			throw runtime_error("Unknown otg type: " + string(type) +
								"in MotionForce task config in config file: " +
								_config_file_name);
		}

		// velocity limits
		const char* max_linear_velocity = otg->Attribute("maxLinearVelocity");
		if (max_linear_velocity) {
			otg_config.max_linear_velocity = stod(max_linear_velocity);
		}

		const char* max_angular_velocity = otg->Attribute("maxAngularVelocity");
		if (max_angular_velocity) {
			otg_config.max_angular_velocity = stod(max_angular_velocity);
		}

		// acceleration limit
		const char* max_linear_acceleration =
			otg->Attribute("maxLinearAcceleration");
		if (max_linear_acceleration) {
			otg_config.max_linear_acceleration = stod(max_linear_acceleration);
		}

		const char* max_angular_acceleration =
			otg->Attribute("maxAngularAcceleration");
		if (max_angular_acceleration) {
			otg_config.max_angular_acceleration =
				stod(max_angular_acceleration);
		}

		// jerk limit
		const char* max_linear_jerk = otg->Attribute("maxLinearJerk");
		if (max_linear_jerk) {
			otg_config.max_linear_jerk = stod(max_linear_jerk);
		}

		const char* max_angular_jerk = otg->Attribute("maxAngularJerk");
		if (max_angular_jerk) {
			otg_config.max_angular_jerk = stod(max_angular_jerk);
		}
		config.otg_config = otg_config;
	}

	// position gains
	tinyxml2::XMLElement* position_gains =
		xml->FirstChildElement("positionGains");
	if (position_gains) {
		config.position_gains_config =
			parseGainsConfig(position_gains, _config_file_name, MOTFORCE_POS);
	}

	// orientation gains
	tinyxml2::XMLElement* orientation_gains =
		xml->FirstChildElement("orientationGains");
	if (orientation_gains) {
		config.orientation_gains_config = parseGainsConfig(
			orientation_gains, _config_file_name, MOTFORCE_ORI);
	}

	// interface config
	tinyxml2::XMLElement* interface = xml->FirstChildElement("interface");
	if (interface) {
		config.interface_config = parseInterfaceConfig(interface);
	}

	return config;
}

}  // namespace SaiInterfaces