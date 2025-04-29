#ifndef SAI_INTERFACES_ROBOT_CONTROLLER_CONFIG_H
#define SAI_INTERFACES_ROBOT_CONTROLLER_CONFIG_H

#include <Eigen/Dense>
#include <map>
#include <string>
#include <variant>

#include "SaiPrimitives.h"
#include "helpers/CommonConfigs.h"

using JointTaskDefaultParams = SaiPrimitives::JointTask::DefaultParameters;
using MotionForceTaskDefaultParams =
	SaiPrimitives::MotionForceTask::DefaultParameters;
using RobotControllerDefaultParams =
	SaiPrimitives::RobotController::DefaultParameters;

namespace SaiInterfaces {

/// @brief Default name of the folder where the log files will be saved
const std::string default_logger_folder_name_robot_controller =
	"log_files/robot_controllers";

/**
 * @brief A config object for the gains of a controller. It contains the P, D,
 * and I gains, as well as a flag to enable gains safety checks (such as making
 * sure that the gains are positive).
 *
 * In the xml config file, several elements will be parsed to this struct:
 * - In JointTaskConfig:
 * 		- \b gains_config from: <gains kp="..." kv="..." ki="..." />
 * - In MotionForceTaskConfig:
 * 		- \b position_gains_config from: <positionGains kp="..." kv="..."
 * ki="..." />
 * 		- \b orientation_gains_config from: <orientationGains kp="..." kv="..."
 * ki="..." />
 * 		- \b force_gains_config from: <forceGains kp="..." kv="..." ki="..." />
 * 		- \b moment_gains_config from: <momentGains kp="..." kv="..." ki="..."
 * />
 *
 * The gains are stored in Eigen::VectorXd objects, which allows for the gains
 * to be either isotropic (same gain in every direction) when the size of the
 * vector is 1, or anisotropic (different gains in different directions) when
 * the size of the vector is greater than 1. If the gains are inosotropic, the
 * size of the vector should correspond to the number of degrees of freedom of
 * the controlled task (i.e. 3 for a position task, n for a joint task on a
 * robot with n degrees of freedom, ...).
 *
 */
struct GainsConfig {
	/// @brief Flag to enable gains safety checks
	bool safety_checks_enabled = true;
	/// @brief Proportional gains
	Eigen::VectorXd kp;
	/// @brief Derivative gains
	Eigen::VectorXd kv;
	/// @brief Integral gains
	Eigen::VectorXd ki;

	/**
	 * @brief Construct a new GainsConfig object with default gains of 0, and
	 * gains vector of size 1.
	 *
	 */
	GainsConfig() {
		kp.setZero(1);
		kv.setZero(1);
		ki.setZero(1);
	}

	/**
	 * @brief Construct a new Gains Config object with vectors of size 1 and
	 * gains values provided as arguments.
	 *
	 * @param kp
	 * @param kv
	 * @param ki
	 */
	GainsConfig(const double kp, const double kv, const double ki) {
		this->kp = Eigen::VectorXd::Constant(1, kp);
		this->kv = Eigen::VectorXd::Constant(1, kv);
		this->ki = Eigen::VectorXd::Constant(1, ki);
	}
};

/**
 * @brief Config for a joint task in the controller.
 *
 * This is parsed from the xml config file, from the following element:
 * 	<jointTask name="..." useDynamicDecoupling="..." bieThreshold="...">
 *      ...
 * 	</jointTask>
 *
 */
struct JointTaskConfig {
	/**
	 * @brief Config for the velocity saturation of the joint task.
	 * If the velocity_limit vector is of size 1, the limits are isotropic. If
	 * the vector is of size n, the limits are different for every joint.
	 *
	 * This is parsed from the xml config file, from the following element:
	 * 	<velocitySaturation enabled="..." velocityLimit="..." />
	 *
	 */
	struct JointVelSatConfig {
		bool enabled = JointTaskDefaultParams::use_velocity_saturation;
		Eigen::VectorXd velocity_limit =
			JointTaskDefaultParams::saturation_velocity *
			Eigen::VectorXd::Ones(1);
	};

	/**
	 * @brief Config for the internal online trajectory generation of the joint
	 * task.
	 *
	 * This is parsed from the xml config file, from the following element:
	 * 	<otg type="..." maxVelocity="..." maxAcceleration="..." maxJerk="..."
	 * />
	 *
	 */
	struct JointOTGConfig {
		/**
		 * @brief Config for the limits of the internal online trajectory
		 * generation of the joint task. If the vectors are of size 1, the
		 * limits are isotropic. If the vectors are of size n, the limits are
		 * different for every joint.
		 *
		 */
		struct JointOTGLimit {
			Eigen::VectorXd max_velocity =
				JointTaskDefaultParams::otg_max_velocity *
				Eigen::VectorXd::Ones(1);
			Eigen::VectorXd max_acceleration =
				JointTaskDefaultParams::otg_max_acceleration *
				Eigen::VectorXd::Ones(1);
			Eigen::VectorXd max_jerk =
				JointTaskDefaultParams::otg_max_jerk * Eigen::VectorXd::Ones(1);
		};

		bool enabled = JointTaskDefaultParams::use_internal_otg;
		bool jerk_limited = JointTaskDefaultParams::internal_otg_jerk_limited;
		JointOTGLimit limits;
	};

	/// @brief Name of the task, parsed from the corresponding attribute of the
	/// <jointTask> element
	std::string task_name = "";
	/// @brief Names of the controlled joints, parsed from the xml config
	/// element <controlledJointNames>...</controlledJointNames>
	std::vector<std::string> controlled_joint_names = {};
	/// @brief Flag to use dynamic decoupling in the joint task, parsed from the
	/// corresponding attribute of the <jointTask> element
	bool use_dynamic_decoupling =
		JointTaskDefaultParams::dynamic_decoupling_type !=
		SaiPrimitives::DynamicDecouplingType::IMPEDANCE;
	/// @brief Threshold for the bounded inertia estimates, parsed from the
	/// corresponding attribute of the <jointTask> element
	double bie_threshold = JointTaskDefaultParams::bie_threshold;

	/// @brief Config for the velocity saturation of the joint task
	JointVelSatConfig velocity_saturation_config;
	/// @brief Config for the internal online trajectory generation of the joint
	JointOTGConfig otg_config;
	/// @brief Config for the gains of the joint task
	GainsConfig gains_config =
		GainsConfig(JointTaskDefaultParams::kp, JointTaskDefaultParams::kv,
					JointTaskDefaultParams::ki);
};

/**
 * @brief Config for a motion force task in the controller.
 *
 * This is parsed from the xml config file, from the following element:
 * 	<motionForceTask name="..." linkName="..." compliantFrame="..."
 * parametrizationInCompliantFrame="..." useDynamicDecoupling="..."
 * bieThreshold="...">
 * 	...
 * 	</motionForceTask>
 *
 */
struct MotionForceTaskConfig {
	/**
	 * @brief Config for a force or moment space parameterization in the motion
	 * force task. This is used to define the dimension of the force or moment
	 * space, and the axis of the force control, or motion control depending on
	 * the force space dimension.
	 *
	 * This is parsed from the xml config file, from the following element:
	 * 	<forceSpaceParametrization forceSpaceDimension="..." axis="..." /> -->
	 * force_space_param_config <momentSpaceParametrization
	 * momentSpaceDimension="..." axis="..." /> --> moment_space_param_config
	 *
	 */
	struct ForceMotionSpaceParamConfig {
		int force_space_dimension;
		Eigen::Vector3d axis;

		ForceMotionSpaceParamConfig()
			: force_space_dimension(0), axis(Eigen::Vector3d::UnitZ()) {}

		ForceMotionSpaceParamConfig(
			const int force_space_dimension,
			const Eigen::Vector3d& axis = Eigen::Vector3d::UnitZ())
			: force_space_dimension(force_space_dimension), axis(axis) {}
	};

	/**
	 * @brief Config for the force control part of the motion force task.
	 *
	 * This is parsed from the xml config file, from the following element:
	 * <forceControl closedLoopForceControl="...">
	 * ...
	 * </forceControl>
	 */
	struct ForceControlConfig {
		/// @brief Flag to enable closed loop force control. Parse from the
		/// corresponding attribute of the <forceControl> element
		bool closed_loop_force_control = false;
		/// @brief Frame of the force sensor with respect to the copliant frame,
		/// parsed from the xml config element
		/// <forceSensorFrame xyz="..." rpy="..." />
		Eigen::Affine3d force_sensor_frame = Eigen::Affine3d::Identity();
		/// @brief Config for the force space parameterization.
		ForceMotionSpaceParamConfig force_space_param_config =
			ForceMotionSpaceParamConfig(
				MotionForceTaskDefaultParams::force_space_dimension);
		/// @brief Config for the moment space parameterization.
		ForceMotionSpaceParamConfig moment_space_param_config =
			ForceMotionSpaceParamConfig(
				MotionForceTaskDefaultParams::moment_space_dimension);
		/// @brief Config for the gains of the force control part of the motion
		GainsConfig force_gains_config =
			GainsConfig(MotionForceTaskDefaultParams::kp_force,
						MotionForceTaskDefaultParams::kv_force,
						MotionForceTaskDefaultParams::ki_force);
		/// @brief Config for the gains of the moment control part of the motion
		GainsConfig moment_gains_config =
			GainsConfig(MotionForceTaskDefaultParams::kp_moment,
						MotionForceTaskDefaultParams::kv_moment,
						MotionForceTaskDefaultParams::ki_moment);
	};

	/**
	 * @brief Config for the velocity saturation of the motion force task.
	 *
	 * This is parsed from the xml config file, from the following element:
	 * 	<velocitySaturation enabled="..." linearVelocityLimit="..."
	 * angularVelocityLimit="..." />
	 *
	 */
	struct VelSatConfig {
		bool enabled = MotionForceTaskDefaultParams::use_velocity_saturation;
		double linear_velocity_limit =
			MotionForceTaskDefaultParams::linear_saturation_velocity;
		double angular_velocity_limit =
			MotionForceTaskDefaultParams::angular_saturation_velocity;
	};

	/**
	 * @brief Config for the internal online trajectory generation of the motion
	 * force task.
	 *
	 * This is parsed from the xml config file, from the following element:
	 * 	<otg type="..." maxLinearVelocity="..." maxAngularVelocity="..."
	 * maxLinearAcceleration="..." maxAngularAcceleration="..."
	 * maxLinearJerk="..." maxAngularJerk="..." />
	 *
	 */
	struct OTGConfig {
		bool enabled = MotionForceTaskDefaultParams::use_internal_otg;
		bool jerk_limited =
			MotionForceTaskDefaultParams::internal_otg_jerk_limited;
		double max_linear_velocity =
			MotionForceTaskDefaultParams::otg_max_linear_velocity;
		double max_angular_velocity =
			MotionForceTaskDefaultParams::otg_max_angular_velocity;
		double max_linear_acceleration =
			MotionForceTaskDefaultParams::otg_max_linear_acceleration;
		double max_angular_acceleration =
			MotionForceTaskDefaultParams::otg_max_angular_acceleration;
		double max_linear_jerk =
			MotionForceTaskDefaultParams::otg_max_linear_jerk;
		double max_angular_jerk =
			MotionForceTaskDefaultParams::otg_max_angular_jerk;
	};

	/**
	 * @brief Config for the motion force task ui interface specific slider
	 * limits. Those are strings that will be written to the webui file
	 * directly.
	 *
	 * This is parsed from the xml config file, from the following element:
	 * 	<interface minGoalPosition='...' maxGoalPosition='...'
	 * minDesiredForce='...' maxDesiredForce='....' minDesiredMoment='...'
	 * maxDesiredMoment='...' />
	 *
	 */
	struct InterfaceConfig {
		std::string min_goal_position = "[-0.7,-0.7,0.0]";
		std::string max_goal_position = "[0.7,0.7,0.9]";
		std::string min_desired_force = "-50";
		std::string max_desired_force = "50";
		std::string min_desired_moment = "-5";
		std::string max_desired_moment = "5";
	};

	/// @brief Name of the task, parsed from the corresponding attribute of the
	/// <motionForceTask> element
	std::string task_name = "";
	/// @brief Name of the link to control, parsed from the corresponding
	/// attribute of the <motionForceTask> element
	std::string link_name = "";
	/// @brief Frame of the compliant task with respect to the link, parsed from
	/// the xml config element <compliantFrame xyz="..." rpy="..." />
	Eigen::Affine3d compliant_frame = Eigen::Affine3d::Identity();
	/// @brief Flag to use a compliant frame in the motion force task, parsed
	/// from the corresponding attribute of the <motionForceTask> element
	bool is_parametrization_in_compliant_frame = false;
	/// @brief Flag to use dynamic decoupling in the motion force task, parsed
	/// from the corresponding attribute of the <motionForceTask> element
	bool use_dynamic_decoupling =
		MotionForceTaskDefaultParams::dynamic_decoupling_type !=
		SaiPrimitives::DynamicDecouplingType::IMPEDANCE;
	/// @brief Threshold for the bounded inertia estimates, parsed from the
	/// corresponding attribute of the <motionForceTask> element
	double bie_threshold = MotionForceTaskDefaultParams::bie_threshold;

	/// @brief Direction in which the task can control motion or force in
	/// translation, parsed from the xml config element
	/// <controlledDirectionsTranslation>...</controlledDirectionsTranslation>
	std::vector<Eigen::Vector3d> controlled_directions_position = {
		Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};
	/// @brief Direction in which the task can control motion or force in
	/// rotation, parsed from the xml config element
	/// <controlledDirectionsRotation>...</controlledDirectionsRotation>
	std::vector<Eigen::Vector3d> controlled_directions_orientation = {
		Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ()};

	/// @brief Config for the force control part of the motion force task
	ForceControlConfig force_control_config;

	/// @brief Config for the velocity saturation of the motion force task
	VelSatConfig velocity_saturation_config;
	/// @brief Config for the internal online trajectory generation of the
	/// motion
	OTGConfig otg_config;
	/// @brief Config for the gains of the position control part of the motion
	GainsConfig position_gains_config =
		GainsConfig(MotionForceTaskDefaultParams::kp_pos,
					MotionForceTaskDefaultParams::kv_pos,
					MotionForceTaskDefaultParams::ki_pos);
	/// @brief Config for the gains of the orientation control part of the
	/// motion
	GainsConfig orientation_gains_config =
		GainsConfig(MotionForceTaskDefaultParams::kp_ori,
					MotionForceTaskDefaultParams::kv_ori,
					MotionForceTaskDefaultParams::ki_ori);

	/// @brief Config for tsk specific ui slider limits
	InterfaceConfig interface_config;
};

/**
 * @brief Configuration for a single controller. Each robot can be pre
 * configured with multiple controllers.
 *
 */
struct RobotSingleControllerConfig {
	/// @brief Name of the controller, parsed from the corresponding attribute
	/// of the <controller> element
	std::string controller_name = "";

	/// @brief Flag to enable gravity compensation in the controller
	bool enable_gravity_compensation =
		RobotControllerDefaultParams::enable_gravity_compensation;
	/// @brief Flag to enable joint limit avoidance in the controller
	bool enable_joint_limit_avoidance =
		RobotControllerDefaultParams::enable_joint_limit_avoidance;
	/// @brief Flag to enable torque saturation in the controller
	bool enable_torque_saturation =
		RobotControllerDefaultParams::enable_torque_saturation;

	/// @brief List of task configurations for this controller in the
	/// hierarchical order
	std::vector<std::variant<JointTaskConfig, MotionForceTaskConfig>>
		tasks_configs = {};
};

/**
 * @brief Config for the RobotControllerRedisInterface. There should be one per
 * robot to control, and each one can contain several controllers.
 *
 * This is parsed from the xml config file, from the following element:
 * 	<robotControlConfiguration robotName="..." robotModelFile="..."
 * redisPrefix="..." controlFrequency="..." >
 * 	...
 * 	</robotControlConfiguration>
 *
 */
struct RobotControllerConfig {
	/// @brief Path to the robot model file, parsed from the corresponding
	/// attribute of the <robotControlConfiguration> element
	std::string robot_model_file = "";
	/// @brief Name of the robot to control, parsed from the corresponding
	/// attribute of the <robotControlConfiguration> element
	std::string robot_name = "";
	/// @brief Base frame of the robot in the world, parsed from the xml config
	/// element <baseFrame xyz="..." rpy="..." />
	Eigen::Affine3d robot_base_in_world = Eigen::Affine3d::Identity();
	/// @brief Gravity vector in the world, parsed from the xml config element
	/// <worldGravity xyz="..." />
	Eigen::Vector3d world_gravity = Eigen::Vector3d(0, 0, -9.81);
	/// @brief Frequency at which the controller will run, parsed from the
	/// corresponding attribute of the <robotControlConfiguration> element
	double control_frequency = 1000.0;

	/// @brief Flag to get the mass matrix from redis, parsed from the
	/// corresponding attribute in the <robotControlConfiguration> element
	/// If true, the mass matrix is read at runtime from redis, as an input to
	/// the controller, similarly to the joint states and velocities. If set to
	/// false, the mass matrix is computed from the urdf mass properties.
	bool get_mass_matrix_from_redis = true;

	/// @brief Config for the logger attached to the
	/// RobotControllerRedisInterface
	LoggerConfig logger_config =
		LoggerConfig(default_logger_folder_name_robot_controller);

	/// @brief Config for the redis interface attached to the robot controller
	RedisConfig redis_config;

	/// @brief Name of the controller to start with, it will initially be the
	/// first controller in the list of controllers in the xml config file
	std::string initial_active_controller_name = "";

	/// @brief Map of the controllers to be used by the robot controller. The
	/// keys are the names of the controllers.
	std::map<std::string, RobotSingleControllerConfig>
		single_controller_configs = {};
};

}  // namespace SaiInterfaces

#endif	// SAI_INTERFACES_ROBOT_CONTROLLER_CONFIG_H
