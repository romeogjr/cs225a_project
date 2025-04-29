#include "HapticDeviceControllerConfigParser.h"
#include "helpers/ConfigParserHelpers.h"

using namespace std;
using namespace Eigen;

namespace SaiInterfaces {

namespace {
HapticDeviceControllerConfig::ControlledRobotTaskConfig
parseControlledRobotTaskConfig(tinyxml2::XMLElement* controlled_robot_task) {
	HapticDeviceControllerConfig::ControlledRobotTaskConfig task;

	if (!controlled_robot_task->Attribute("robotName") ||
		!controlled_robot_task->Attribute("controllerName") ||
		!controlled_robot_task->Attribute("taskName")) {
		throw runtime_error(
			"Each controlledRobotTaskConfig must have a robotName attribute in "
			"hapticDeviceControllerConfig");
	}

	task.robot_name = controlled_robot_task->Attribute("robotName");
	task.controller_name = controlled_robot_task->Attribute("controllerName");
	task.task_name = controlled_robot_task->Attribute("taskName");

	if (task.robot_name.empty() || task.controller_name.empty() ||
		task.task_name.empty()) {
		throw runtime_error(
			"The controlledRobotTaskConfig must have a non empty robotName, "
			"controllerName, and taskName attribute in "
			"hapticDeviceControllerConfig");
	}

	return task;
}

HapticDeviceControllerConfig::HomingModeConfig parseHomingModeConfig(
	tinyxml2::XMLElement* homing) {
	HapticDeviceControllerConfig::HomingModeConfig homing_params;

	if (homing->Attribute("maxLinearVelocity")) {
		homing_params.max_linvel = homing->DoubleAttribute("maxLinearVelocity");
	}
	if (homing->Attribute("maxAngularVelocity")) {
		homing_params.max_angvel =
			homing->DoubleAttribute("maxAngularVelocity");
	}

	return homing_params;
}

HapticDeviceControllerConfig::ImpedanceModeConfig::VirtualWorkspaceLimitsConfig
parseVirtualWorkspaceLimitsConfig(
	tinyxml2::XMLElement* virtual_workspace_limits) {
	HapticDeviceControllerConfig::ImpedanceModeConfig::
		VirtualWorkspaceLimitsConfig limits;

	if (virtual_workspace_limits->Attribute("enabled")) {
		limits.enabled = virtual_workspace_limits->BoolAttribute("enabled");
	}
	if (virtual_workspace_limits->Attribute("radius")) {
		limits.radius_limit =
			virtual_workspace_limits->DoubleAttribute("radius");
	}
	if (virtual_workspace_limits->Attribute("angle")) {
		limits.angle_limit = virtual_workspace_limits->DoubleAttribute("angle");
	}

	return limits;
}

HapticDeviceControllerConfig::ImpedanceModeConfig::ScalingFactorsConfig
parseScalingFactorsConfig(tinyxml2::XMLElement* scaling_factors) {
	HapticDeviceControllerConfig::ImpedanceModeConfig::ScalingFactorsConfig
		scaling;

	if (scaling_factors->Attribute("linear")) {
		scaling.scaling_factor_pos = scaling_factors->DoubleAttribute("linear");
	}
	if (scaling_factors->Attribute("angular")) {
		scaling.scaling_factor_ori =
			scaling_factors->DoubleAttribute("angular");
	}

	return scaling;
}

HapticDeviceControllerConfig::ImpedanceModeConfig::VariableDampingConfig
parseVariableDampingConfig(tinyxml2::XMLElement* variable_damping) {
	HapticDeviceControllerConfig::ImpedanceModeConfig::VariableDampingConfig
		damping;

	if (variable_damping->Attribute("linearVelocityThresholds")) {
		damping.linvel_thresholds = ConfigParserHelpers::parseVectorXd(
			variable_damping->Attribute("linearVelocityThresholds"));
	}
	if (variable_damping->Attribute("angularVelocityThresholds")) {
		damping.angvel_thresholds = ConfigParserHelpers::parseVectorXd(
			variable_damping->Attribute("angularVelocityThresholds"));
	}
	if (variable_damping->Attribute("linearDamping")) {
		damping.linear_damping = ConfigParserHelpers::parseVectorXd(
			variable_damping->Attribute("linearDamping"));
	}
	if (variable_damping->Attribute("angularDamping")) {
		damping.angular_damping = ConfigParserHelpers::parseVectorXd(
			variable_damping->Attribute("angularDamping"));
	}

	if (damping.linvel_thresholds.size() != damping.linear_damping.size() ||
		damping.angvel_thresholds.size() != damping.angular_damping.size()) {
		throw runtime_error(
			"linearVelocityThresholds and linearDamping must have the same "
			"size, and "
			"angularVelocityThresholds and angularDamping must have the same "
			"size in "
			"hapticDeviceControllerConfig");
	}

	return damping;
}

HapticDeviceControllerConfig::ImpedanceModeConfig::ForceFeedbackConfig
parseForceFeedbackConfig(tinyxml2::XMLElement* force_feedback) {
	HapticDeviceControllerConfig::ImpedanceModeConfig::ForceFeedbackConfig
		force_feedback_config;

	if (force_feedback->Attribute("reductionFactorForce")) {
		force_feedback_config.reduction_factor_force =
			force_feedback->DoubleAttribute("reductionFactorForce");
	}
	if (force_feedback->Attribute("reductionFactorMoment")) {
		force_feedback_config.reduction_factor_moment =
			force_feedback->DoubleAttribute("reductionFactorMoment");
	}
	if (force_feedback->Attribute("proxyForceSpaceDimension")) {
		force_feedback_config.proxy_force_space_dimension =
			force_feedback->IntAttribute("proxyForceSpaceDimension");
	}
	if (force_feedback->Attribute("proxyMomentSpaceDimension")) {
		force_feedback_config.proxy_moment_space_dimension =
			force_feedback->IntAttribute("proxyMomentSpaceDimension");
	}
	if (force_feedback->Attribute("proxyForceAxis")) {
		force_feedback_config.proxy_force_axis =
			ConfigParserHelpers::parseVector3d(
				force_feedback->Attribute("proxyForceAxis"));
	}
	if (force_feedback->Attribute("proxyMomentAxis")) {
		force_feedback_config.proxy_moment_axis =
			ConfigParserHelpers::parseVector3d(
				force_feedback->Attribute("proxyMomentAxis"));
	}

	if (force_feedback_config.reduction_factor_force < 0 ||
		force_feedback_config.reduction_factor_force > 1 ||
		force_feedback_config.reduction_factor_moment < 0 ||
		force_feedback_config.reduction_factor_moment > 1) {
		throw runtime_error(
			"Reduction factors must be between 0 and 1 in forceFeedbackConfig "
			"in hapticDeviceControllerConfig");
	}

	if (force_feedback_config.proxy_force_space_dimension < 0 ||
		force_feedback_config.proxy_force_space_dimension > 3 ||
		force_feedback_config.proxy_moment_space_dimension < 0 ||
		force_feedback_config.proxy_moment_space_dimension > 3) {
		throw runtime_error(
			"Proxy force_feedback_config space dimension must be between 0 and "
			"3 in forceFeedbackConfig in hapticDeviceControllerConfig");
	}

	if (force_feedback_config.proxy_force_axis.norm() < 1e-3) {
		throw runtime_error(
			"Proxy force force_feedback_config axis must be non-zero in "
			"forceFeedbackConfig in hapticDeviceControllerConfig");
	}
	force_feedback_config.proxy_force_axis.normalize();

	if (force_feedback_config.proxy_moment_axis.norm() < 1e-3) {
		throw runtime_error(
			"Proxy moment force_feedback_config axis must be non-zero in "
			"forceFeedbackConfig in hapicDeviceControllerConfig");
	}
	force_feedback_config.proxy_moment_axis.normalize();

	return force_feedback_config;
}

HapticDeviceControllerConfig::ImpedanceModeConfig parseImpedanceModeConfig(
	tinyxml2::XMLElement* impedance_mode) {
	HapticDeviceControllerConfig::ImpedanceModeConfig impedance_mode_config;

	tinyxml2::XMLElement* virtual_workspace_limits =
		impedance_mode->FirstChildElement("virtualWorkspaceLimits");
	if (virtual_workspace_limits) {
		impedance_mode_config.virtual_workspace_limits =
			parseVirtualWorkspaceLimitsConfig(virtual_workspace_limits);
	}

	tinyxml2::XMLElement* scaling_factors =
		impedance_mode->FirstChildElement("scalingFactors");
	if (scaling_factors) {
		impedance_mode_config.scaling_factors =
			parseScalingFactorsConfig(scaling_factors);
	}

	tinyxml2::XMLElement* variable_damping =
		impedance_mode->FirstChildElement("variableDamping");
	if (variable_damping) {
		impedance_mode_config.variable_damping =
			parseVariableDampingConfig(variable_damping);
	}

	tinyxml2::XMLElement* force_feedback =
		impedance_mode->FirstChildElement("forceFeedback");
	if (force_feedback) {
		impedance_mode_config.force_feedback =
			parseForceFeedbackConfig(force_feedback);
	}

	return impedance_mode_config;
}

HapticDeviceControllerConfig::AdmittanceModeConfig parseAdmittanceModeConfig(
	tinyxml2::XMLElement* admittance) {
	HapticDeviceControllerConfig::AdmittanceModeConfig admittance_mode_config;

	if (admittance->Attribute("linearAdmittanceFactor")) {
		admittance_mode_config.device_force_to_robot_delta_position =
			admittance->DoubleAttribute("linearAdmittanceFactor");
	}
	if (admittance->Attribute("angularAdmittanceFactor")) {
		admittance_mode_config.device_moment_to_robot_delta_orientation =
			admittance->DoubleAttribute("angularAdmittanceFactor");
	}
	if (admittance->Attribute("forceDeadband")) {
		admittance_mode_config.force_deadband =
			admittance->DoubleAttribute("forceDeadband");
	}
	if (admittance->Attribute("momentDeadband")) {
		admittance_mode_config.moment_deadband =
			admittance->DoubleAttribute("momentDeadband");
	}

	return admittance_mode_config;
}

HapticDeviceControllerConfig::VirtualGuidanceConfig parseVirtualGuidanceConfig(
	tinyxml2::XMLElement* virtual_guidance) {
	HapticDeviceControllerConfig::VirtualGuidanceConfig guidance_config;

	if (virtual_guidance->Attribute("enabled")) {
		guidance_config.enabled = virtual_guidance->BoolAttribute("enabled");
	}
	if (virtual_guidance->Attribute("originToCurrentPosition")) {
		guidance_config.set_origin_to_current_position =
			virtual_guidance->BoolAttribute("originToCurrentPosition");
	}
	if (virtual_guidance->Attribute("origin")) {
		guidance_config.origin_point = ConfigParserHelpers::parseVector3d(
			virtual_guidance->Attribute("origin"));
	}
	if (virtual_guidance->Attribute("axis")) {
		guidance_config.axis = ConfigParserHelpers::parseVector3d(
			virtual_guidance->Attribute("axis"));
	}

	if (guidance_config.axis.norm() < 1e-3) {
		throw runtime_error(
			"Virtual guidance normal direction must be non-zero in "
			"virtualGuidanceConfig in hapicDeviceControllerConfig");
	}
	guidance_config.axis.normalize();

	return guidance_config;
}

HapticDeviceControllerConfig::GainsConfig parseGainsConfig(
	tinyxml2::XMLElement* xml) {
	HapticDeviceControllerConfig::GainsConfig gains;

	if (xml->Attribute("kpPos")) {
		gains.kp_pos = xml->DoubleAttribute("kpPos");
	}
	if (xml->Attribute("kvPos")) {
		gains.kv_pos = xml->DoubleAttribute("kvPos");
	}
	if (xml->Attribute("kpOri")) {
		gains.kp_ori = xml->DoubleAttribute("kpOri");
	}
	if (xml->Attribute("kvOri")) {
		gains.kv_ori = xml->DoubleAttribute("kvOri");
	}

	if (gains.kp_pos < 0 || gains.kv_pos < 0 || gains.kp_ori < 0 ||
		gains.kv_ori < 0) {
		throw runtime_error(
			"all kp and kv must be positive in gainsConfig in "
			"hapticDeviceControlConfiguration");
	}

	return gains;
}

}  // namespace

std::vector<HapticDeviceControllerConfig>
HapticDeviceControllerConfigParser::parseConfig(
	const std::string& config_file) {
	_config_file_name = config_file;
	std::vector<HapticDeviceControllerConfig> configs;

	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(config_file.c_str()) != tinyxml2::XML_SUCCESS) {
		throw runtime_error(
			"Could not load haptic device controller config file: " +
			config_file);
	}

	if (doc.FirstChildElement("hapticDeviceControlConfiguration") == nullptr) {
		return configs;
	}

	// loop over the hapticDeviceControlConfiguration elements
	for (tinyxml2::XMLElement* hapticDeviceControlConfiguration =
			 doc.FirstChildElement("hapticDeviceControlConfiguration");
		 hapticDeviceControlConfiguration;
		 hapticDeviceControlConfiguration =
			 hapticDeviceControlConfiguration->NextSiblingElement(
				 "hapticDeviceControlConfiguration")) {
		configs.push_back(parseControllerConfig(
			hapticDeviceControlConfiguration, configs.size()));
	}

	return configs;
}

HapticDeviceControllerConfig
HapticDeviceControllerConfigParser::parseControllerConfig(
	tinyxml2::XMLElement* controlConfiguration, const int device_number) {
	HapticDeviceControllerConfig config;

	config.device_id = device_number;

	// extract controlled robot task
	tinyxml2::XMLElement* controlled_robot_task =
		controlConfiguration->FirstChildElement("controlledRobotTask");
	if (!controlled_robot_task) {
		throw runtime_error(
			"Each haptic device controller must have a "
			"controlledRobotTaskConfig "
			"element in config file: " +
			_config_file_name);
	}
	const auto controlled_robot_task_config =
		parseControlledRobotTaskConfig(controlled_robot_task);
	if (std::find(
			_controlled_robot_tasks.begin(), _controlled_robot_tasks.end(),
			controlled_robot_task_config) != _controlled_robot_tasks.end()) {
		throw runtime_error(
			"Each hapticDeviceControlConfiguration must have a unique "
			"controlledRobotTaskConfig in config file: " +
			_config_file_name);
	}
	_controlled_robot_tasks.push_back(controlled_robot_task_config);
	config.controlled_robot_task = controlled_robot_task_config;

	// extract control frequency
	if (controlConfiguration->Attribute("controlFrequency")) {
		config.control_frequency =
			controlConfiguration->DoubleAttribute("controlFrequency");
	}

	// extract device base frame
	tinyxml2::XMLElement* base_frame =
		controlConfiguration->FirstChildElement("baseFrame");
	if (base_frame) {
		config.haptic_device_base_in_world =
			ConfigParserHelpers::parsePose(base_frame);
	}

	// control mode
	if (controlConfiguration->Attribute("controlMode")) {
		string control_mode_str =
			controlConfiguration->Attribute("controlMode");
		if (control_mode_str == "impedance") {
			config.control_mode =
				HapticDeviceControllerConfig::ControlMode::IMPEDANCE;
		} else if (control_mode_str == "admittance") {
			config.control_mode =
				HapticDeviceControllerConfig::ControlMode::ADMITTANCE;
		} else {
			throw runtime_error(
				"Unknown control mode: " + control_mode_str +
				" in config file: " + _config_file_name +
				"\nsupported modes are impedance and admittance");
		}
	}

	// switch usage type
	if (controlConfiguration->Attribute("switchUsageType")) {
		string switch_usage_type_str =
			controlConfiguration->Attribute("switchUsageType");
		if (switch_usage_type_str == "click") {
			config.switch_usage_type =
				HapticDeviceControllerConfig::SwitchUsageType::CLICK;
		} else if (switch_usage_type_str == "hold") {
			config.switch_usage_type =
				HapticDeviceControllerConfig::SwitchUsageType::HOLD;
		} else {
			throw runtime_error(
				"Unknown switch usage type: " + switch_usage_type_str +
				" in config file: " + _config_file_name +
				"\nsupported types are click and hold");
		}
	}

	// switch function
	if (controlConfiguration->Attribute("switchFunction")) {
		string switch_function_str =
			controlConfiguration->Attribute("switchFunction");
		if (switch_function_str == "clutch") {
			config.switch_function =
				HapticDeviceControllerConfig::SwitchFunction::CLUTCH;
		} else if (switch_function_str == "orientationControl") {
			config.switch_function = HapticDeviceControllerConfig::
				SwitchFunction::ORIENTATION_CONTROL;
		} else {
			throw runtime_error(
				"Unknown switch function: " + switch_function_str +
				" in config file: " + _config_file_name +
				"\nsupported functions are clutch and orientationControl");
		}
	}

	// orientation teleop enabled
	if (controlConfiguration->Attribute("orientationTeleopEnabled")) {
		config.orientation_teleop_enabled =
			controlConfiguration->BoolAttribute("orientationTeleopEnabled");
	}

	// use switch to exit homing
	if (controlConfiguration->Attribute("useSwitchToExitHoming")) {
		config.use_switch_to_exit_homing =
			controlConfiguration->BoolAttribute("useSwitchToExitHoming");
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
			logger, default_logger_folder_name_haptic_controller);
	}

	// homing parameters
	tinyxml2::XMLElement* homing =
		controlConfiguration->FirstChildElement("homingMode");
	if (homing) {
		config.homing_mode_config = parseHomingModeConfig(homing);
	}

	// impedance mode config
	tinyxml2::XMLElement* impedance_mode =
		controlConfiguration->FirstChildElement("impedanceMode");
	if (impedance_mode) {
		config.impedance_mode_config = parseImpedanceModeConfig(impedance_mode);
	}

	// admittance mode config
	tinyxml2::XMLElement* admittance_mode =
		controlConfiguration->FirstChildElement("admittanceMode");
	if (admittance_mode) {
		config.admittance_mode_config =
			parseAdmittanceModeConfig(admittance_mode);
	}

	// plane guidance config
	tinyxml2::XMLElement* plane_guidance =
		controlConfiguration->FirstChildElement("planeGuidance");
	if (plane_guidance) {
		config.plane_guidance_config =
			parseVirtualGuidanceConfig(plane_guidance);
	}

	// line guidance config
	tinyxml2::XMLElement* line_guidance =
		controlConfiguration->FirstChildElement("lineGuidance");
	if (line_guidance) {
		config.line_guidance_config = parseVirtualGuidanceConfig(line_guidance);
	}

	// control gains
	tinyxml2::XMLElement* control_gains_config =
		controlConfiguration->FirstChildElement("controlGains");
	if (control_gains_config) {
		config.control_gains_config = parseGainsConfig(control_gains_config);
	}

	// guidance gains
	tinyxml2::XMLElement* guidance_gains_config =
		controlConfiguration->FirstChildElement("guidanceGains");
	if (guidance_gains_config) {
		config.guidance_gains_config = parseGainsConfig(guidance_gains_config);
	}

	return config;
}

}  // namespace SaiInterfaces