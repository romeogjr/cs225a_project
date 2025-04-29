#ifndef SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_CONFIG_H
#define SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_CONFIG_H

#include <Eigen/Dense>
#include <optional>

#include "SaiPrimitives.h"
#include "helpers/CommonConfigs.h"

using HapticControlDefaultParameters =
	SaiPrimitives::HapticDeviceController::DefaultParameters;

namespace SaiInterfaces {

const std::string default_logger_folder_name_haptic_controller =
	"log_files/haptic_controllers";

struct HapticDeviceControllerConfig {
	enum SwitchUsageType { CLICK, HOLD };
	enum SwitchFunction { CLUTCH, ORIENTATION_CONTROL };
	enum ControlMode { IMPEDANCE, ADMITTANCE };

	struct ControlledRobotTaskConfig {
		std::string robot_name = "";
		std::string controller_name = "";
		std::string task_name = "";

		bool operator==(const ControlledRobotTaskConfig& other) const {
			return (robot_name == other.robot_name) &&
				   (controller_name == other.controller_name) &&
				   (task_name == other.task_name);
		}
	};

	struct HomingModeConfig {
		double max_linvel = HapticControlDefaultParameters::homing_max_linvel;
		double max_angvel = HapticControlDefaultParameters::homing_max_angvel;
	};

	struct ImpedanceModeConfig {
		struct VirtualWorkspaceLimitsConfig {
			bool enabled = false;
			double radius_limit =
				HapticControlDefaultParameters::device_workspace_radius_limit;
			double angle_limit =
				HapticControlDefaultParameters::device_workspace_angle_limit;
		};

		struct ScalingFactorsConfig {
			double scaling_factor_pos =
				HapticControlDefaultParameters::scaling_factor_pos;
			double scaling_factor_ori =
				HapticControlDefaultParameters::scaling_factor_ori;
		};

		struct VariableDampingConfig {
			VectorXd linvel_thresholds = VectorXd::Zero(1);
			VectorXd angvel_thresholds = VectorXd::Zero(1);
			VectorXd linear_damping = VectorXd::Zero(1);
			VectorXd angular_damping = VectorXd::Zero(1);
		};

		struct ForceFeedbackConfig {
			double reduction_factor_force =
				HapticControlDefaultParameters::reduction_factor_force;
			double reduction_factor_moment =
				HapticControlDefaultParameters::reduction_factor_moment;
			int proxy_force_space_dimension = 0;
			int proxy_moment_space_dimension = 0;
			Vector3d proxy_force_axis = Vector3d::UnitZ();
			Vector3d proxy_moment_axis = Vector3d::UnitZ();
		};

		ScalingFactorsConfig scaling_factors;
		ForceFeedbackConfig force_feedback;
		VariableDampingConfig variable_damping;
		VirtualWorkspaceLimitsConfig virtual_workspace_limits;
	};

	struct AdmittanceModeConfig {
		double device_force_to_robot_delta_position =
			HapticControlDefaultParameters::
				device_force_to_robot_delta_position;
		double device_moment_to_robot_delta_orientation =
			HapticControlDefaultParameters::
				device_moment_to_robot_delta_orientation;
		double force_deadband = HapticControlDefaultParameters::force_deadband;
		double moment_deadband =
			HapticControlDefaultParameters::moment_deadband;
	};

	struct VirtualGuidanceConfig {
		bool enabled = false;
		bool set_origin_to_current_position = true;
		Vector3d origin_point = Vector3d::Zero();
		Vector3d axis = Vector3d::UnitZ();
	};

	struct GainsConfig {
		double kp_pos = 0.0;
		double kv_pos = 0.0;
		double kp_ori = 0.0;
		double kv_ori = 0.0;
	};

	int device_id = 0;

	double control_frequency = 1000.0;
	Affine3d haptic_device_base_in_world = Affine3d::Identity();

	ControlledRobotTaskConfig controlled_robot_task;

	ControlMode control_mode = ControlMode::IMPEDANCE;
	SwitchUsageType switch_usage_type = SwitchUsageType::CLICK;
	SwitchFunction switch_function = SwitchFunction::CLUTCH;
	bool orientation_teleop_enabled = false;
	bool use_switch_to_exit_homing = true;

	LoggerConfig logger_config =
		LoggerConfig(default_logger_folder_name_haptic_controller);
	RedisConfig redis_config;

	HomingModeConfig homing_mode_config;
	ImpedanceModeConfig impedance_mode_config;
	AdmittanceModeConfig admittance_mode_config;

	VirtualGuidanceConfig plane_guidance_config;
	VirtualGuidanceConfig line_guidance_config;

	std::optional<GainsConfig> control_gains_config;
	std::optional<GainsConfig> guidance_gains_config;
};

}  // namespace SaiInterfaces

#endif	// SAI_INTERFACES_HAPTIC_DEVICE_CONTROLLER_CONFIG_H