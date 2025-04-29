#ifndef SAI_INTERFACES_SIMVIZ_CONFIG_H
#define SAI_INTERFACES_SIMVIZ_CONFIG_H

#include <Eigen/Dense>
#include <map>
#include <string>

#include "helpers/CommonConfigs.h"

namespace SaiInterfaces {

/// @brief Default name of the folder where the log files will be saved
const std::string default_logger_folder_name_simviz = "log_files/simviz";

/// @brief Enum to specify the mode of the simulation and visualization
enum SimVizMode { SIMVIZ = 0, SIM_ONLY = 1, VIZ_ONLY = 2 };

/**
 * @brief Configuration struct for the dynamic and rendering parameters of a
 * specific robot or object
 *
 * This is parsed from the xml file from the following element:
 * 	<robotOrObjectSpecificParameters name="..." dynamicsEnabled="..."
 * renderingEnabled="..." jointLimitsEnabled="..." frictionCoefficient="..."
 * collisionRestitutionCoefficient="..." wireMeshRenderingMode="..."
 * framesRenderingEnabled="..." frameSizeWhenRendering="..." />
 *
 */
struct DynamicAndRenderingParams {
	/// @brief Whether the dynamics are enabled for this robot or object (can
	/// interact with the world or not)
	bool dynamics_enabled = true;
	/// @brief Whether the rendering is enabled for this robot or object
	/// (appears in the visualization or not)
	bool rendering_enabled = true;
	/// @brief Whether the joint limits are enabled for this robot in simulation
	/// (no effect on objects)
	bool joint_limits_enabled = true;
	/// @brief The coefficient of restitution for collisions
	double collision_restitution_coefficient = 0.0;
	/// @brief The coefficient of friction for collisions
	double friction_coefficient = 0.5;
	/// @brief Whether to render the object in wire mesh mode
	bool wire_mesh_rendering_mode = false;
	/// @brief Whether to render the kinematic frames of the robot or object
	bool frames_rendering_enabled = false;
	/// @brief The size of the kinematic frames when rendering
	double frames_size_when_rendering = 0.2;

	bool operator==(const DynamicAndRenderingParams& other) const {
		return (dynamics_enabled == other.dynamics_enabled) &&
			   (rendering_enabled == other.rendering_enabled) &&
			   (joint_limits_enabled == other.joint_limits_enabled) &&
			   (collision_restitution_coefficient ==
				other.collision_restitution_coefficient) &&
			   (friction_coefficient == other.friction_coefficient) &&
			   (wire_mesh_rendering_mode == other.wire_mesh_rendering_mode) &&
			   (frames_rendering_enabled == other.frames_rendering_enabled) &&
			   (frames_size_when_rendering == other.frames_size_when_rendering);
	}
};

/**
 * @brief Configuration struct for a force sensor defined in the simulation
 *
 * This is parsed from the xml file from the following element:
 * 	<forceSensor robotOrObjectName="..." linkName="..." cutoffFrequency="...">
 * 		<origin xyz="..." rpy="..." />
 * 	</forceSensor>
 *
 */
struct SimForceSensorConfig {
	/// @brief The name of the robot or object to which the force sensor is
	/// attached
	std::string robot_or_object_name = "";
	/// @brief The name of the link to which the force sensor is attached. Leave
	/// empty for objects
	std::string link_name = "";
	/// @brief The pose of the force sensor in the link frame
	Eigen::Affine3d transform_in_link = Eigen::Affine3d::Identity();
	/// @brief The cutoff frequency of the force sensor filter. Leave at 0 for
	/// no filtering
	double cutoff_frequency = 0.0;

	bool operator==(const SimForceSensorConfig& other) const {
		return (robot_or_object_name == other.robot_or_object_name) &&
			   (link_name == other.link_name) &&
			   (transform_in_link.matrix() ==
				other.transform_in_link.matrix()) &&
			   (cutoff_frequency == other.cutoff_frequency);
	}
};

/**
 * @brief Main configuration struct for the simulation and visualization.
 *
 * This is parsed from the xml file from the following element:
 * 	<simviz worldFile="..." mode="..." redisPrefix="...">
 *  ...
 *  </simviz>
 *
 */
struct SimVizConfig {
	/// @brief The path to the world file to be loaded in the simulation
	std::string world_file = "";
	/// @brief Whether to enable the robot joint limits in the simulation
	bool enable_joint_limits = true;
	/// @brief Whether to enable gravity compensation for the robots by the
	/// simulation
	bool enable_gravity_compensation = true;
	/// @brief The global friction coefficient for the simulation
	double global_friction_coefficient = 0.5;
	/// @brief The global coefficient of restitution for collisions in the
	/// simulation
	double global_collision_restitution = 0.0;
	/// @brief The timestep of the simulation
	double timestep = 0.001;
	/// @brief The speedup factor of the simulation
	double speedup_factor = 1.0;

	/// @brief Whether to publish the mass matrices of the robots to redis
	bool publish_mass_matrices_to_redis = true;

	/// @brief The dynamic and rendering parameters for each robot or object in
	/// the simulation.
	std::map<std::string, DynamicAndRenderingParams>
		model_specific_dynamic_and_rendering_params = {};

	/// @brief The mode of the simulation and visualization. Supported modes:
	/// 	- SIMVIZ: Run both simulation and visualization
	/// 	- SIM_ONLY: Run only the simulation
	/// 	- VIZ_ONLY: Run only the visualization
	SimVizMode mode = SimVizMode::SIMVIZ;

	/// @brief The force sensors defined in the simulation
	std::vector<SimForceSensorConfig> force_sensors = {};

	/// @brief The logger configuration for the simulation and visualization
	LoggerConfig logger_config =
		LoggerConfig(default_logger_folder_name_simviz);

	/// @brief The redis configuration for the simulation and visualization
	RedisConfig redis_config;

	bool operator==(const SimVizConfig& other) const {
		return (world_file == other.world_file) &&
			   (enable_joint_limits == other.enable_joint_limits) &&
			   (global_friction_coefficient ==
				other.global_friction_coefficient) &&
			   (global_collision_restitution ==
				other.global_collision_restitution) &&
			   (timestep == other.timestep) && (mode == other.mode) &&
			   (force_sensors == other.force_sensors) &&
			   (logger_config == other.logger_config);
	}
};

}  // namespace SaiInterfaces

#endif	// SAI_INTERFACES_SIMVIZ_CONFIG_H
