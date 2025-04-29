#include "HapticDeviceControllerRedisInterface.h"

#include "redis/keys/chai_haptic_devices_driver.h"

using namespace SaiCommon::ChaiHapticDriverKeys;

namespace SaiInterfaces {
namespace {

bool external_stop_signal = false;
void stop(int i) { external_stop_signal = true; }

const std::string reset_inputs_redis_group = "reset_input_group";

}  // namespace

HapticDeviceControllerRedisInterface::HapticDeviceControllerRedisInterface(
	const HapticDeviceControllerConfig& config,
	const bool setup_signal_handler) {
	_config = config;

	_logging_on = _config.logger_config.start_with_logger_on;
	_logging_state = _logging_on ? LoggingState::START : LoggingState::OFF;

	_redis_client = make_unique<SaiCommon::RedisClient>(
		_config.redis_config.redis_namespace_prefix);
	_redis_client->connect(_config.redis_config.redis_ip,
						   _config.redis_config.redis_port);

	initialize();

	if (setup_signal_handler) {
		signal(SIGABRT, &stop);
		signal(SIGTERM, &stop);
		signal(SIGINT, &stop);
	}
}

void HapticDeviceControllerRedisInterface::run(
	const std::atomic<bool>& user_stop_signal) {
	// let redis know the controller is running for that device
	_redis_client->setBool("haptic_controllers::device" +
							   std::to_string(_config.device_id) +
							   "::is_running",
						   true);

	// start redis communication thread
	std::thread redis_communication_thread(
		&HapticDeviceControllerRedisInterface::runRedisCommunication, this,
		std::ref(user_stop_signal));

	// create timer
	SaiCommon::LoopTimer timer(_config.control_frequency);
	timer.setTimerName(
		"HapticDeviceControllerRedisInterface Timer for controller on "
		"device: " +
		std::to_string(_config.device_id));

	while (!user_stop_signal && !external_stop_signal) {
		timer.waitForNextLoop();

		// process inputs
		processInputs();

		// compute haptic control
		SaiPrimitives::HapticControllerOutput haptic_output =
			_haptic_controller->computeHapticControl(_haptic_input);

		// compute torques
		{
			lock_guard<mutex> lock(_haptic_output_mutex);
			_haptic_output = haptic_output;
		}
	}
	timer.stop();

	// stop logging
	_logger->stop();

	// stop redis communication
	redis_communication_thread.join();

	// let redis know the controller is no longer running for that device
	_redis_client->setBool("haptic_controllers::device" +
							   std::to_string(_config.device_id) +
							   "::is_running",
						   false);

	// reset commanded forces and torques to zero
	_redis_client->setEigen(
		createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, _config.device_id),
		Eigen::Vector3d::Zero());
	_redis_client->setEigen(
		createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, _config.device_id),
		Eigen::Vector3d::Zero());
	_redis_client->setDouble(
		createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX, _config.device_id),
		0.0);

	timer.printInfoPostRun();
}

void HapticDeviceControllerRedisInterface::initialize() {
	// wait for device driver to start
	unsigned long long display_counter = 0;
	const string driver_running_key =
		createRedisKey(DRIVER_RUNNING_KEY_SUFFIX, _config.device_id);
	while (!_redis_client->exists(driver_running_key) ||
		   !_redis_client->getBool(driver_running_key)) {
		if (display_counter % 50 == 0) {
			cout << "Waiting for haptic device driver to start" << endl;
		}
		display_counter++;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if (display_counter > 500) {
			throw std::runtime_error(
				"Haptic device controller redis interface: "
				"timed out while waiting for the haptic device driver to start "
				"for device [" +
				std::to_string(_config.device_id) + "]");
		}
	}
	cout << "Driver started" << endl;

	// wait for robot controller to start
	display_counter = 0;
	const string robot_controller_running_key =
		"controllers::" + _config.controlled_robot_task.robot_name +
		"::is_running";
	while (!_redis_client->exists(robot_controller_running_key) ||
		   !_redis_client->getBool(robot_controller_running_key)) {
		if (display_counter % 50 == 0) {
			cout << "Waiting for robot controller to start" << endl;
		}
		display_counter++;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if (display_counter > 500) {
			throw std::runtime_error(
				"Haptic device controller redis interface: "
				"timed out while waiting for the robot controller to start for "
				"robot [" +
				_config.controlled_robot_task.robot_name + "]");
		}
	}
	cout << "Robot controller started" << endl;

	// wait for robot controller to be active
	display_counter = 0;
	const string robot_controller_active_key =
		"controllers::" + _config.controlled_robot_task.robot_name +
		"::active_controller_name";
	while (!_redis_client->exists(robot_controller_active_key) ||
		   _redis_client->get(robot_controller_active_key) !=
			   _config.controlled_robot_task.controller_name) {
		if (display_counter % 50 == 0) {
			cout << "Waiting for robot controller ["
				 << _config.controlled_robot_task.controller_name
				 << "] to be active" << endl;
		}
		display_counter++;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if (display_counter > 500) {
			throw std::runtime_error(
				"Haptic device controller redis interface: "
				"timed out while waiting for the robot controller [" +
				_config.controlled_robot_task.controller_name +
				"] to be active for robot [" +
				_config.controlled_robot_task.robot_name + "]");
		}
	}
	cout << "Robot controller active" << endl;

	// get task position and orientation
	const string robot_task_redis_key_prefix =
		"controllers::" + _config.controlled_robot_task.robot_name +
		"::" + _config.controlled_robot_task.controller_name +
		"::" + _config.controlled_robot_task.task_name + "::";
	const string current_position_key =
		robot_task_redis_key_prefix + "current_position";
	const string current_orientation_key =
		robot_task_redis_key_prefix + "current_orientation";
	if (!_redis_client->exists(current_position_key) ||
		!_redis_client->exists(current_orientation_key)) {
		throw std::runtime_error(
			"Haptic device controller redis interface: "
			"could not find task named [" +
			_config.controlled_robot_task.task_name +
			"] in robot controller [" +
			_config.controlled_robot_task.controller_name + "] for robot [" +
			_config.controlled_robot_task.robot_name + "]");
	}

	// wait for robot to have a non-zero position, which means the controller is
	// initialized
	display_counter = 0;
	while (_redis_client->getEigen(current_position_key).norm() == 0.0) {
		if (display_counter % 50 == 0) {
			cout << "Waiting for robot task to be initialized" << endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		display_counter++;
		if (display_counter > 500) {
			throw std::runtime_error(
				"Haptic device controller redis interface: "
				"timed out while waiting for the robot task to be initialized");
		}
	}

	SaiPrimitives::HapticDeviceController::DeviceLimits device_limits(
		_redis_client->getEigen(
			createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, _config.device_id)),
		_redis_client->getEigen(
			createRedisKey(MAX_DAMPING_KEY_SUFFIX, _config.device_id)),
		_redis_client->getEigen(
			createRedisKey(MAX_FORCE_KEY_SUFFIX, _config.device_id)));

	// setup switch state receiver
	_redis_client->setBool(
		createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, _config.device_id),
		true);
	_haptic_device_switch_pressed = false;
	_haptic_device_switch_was_pressed = false;
	_redis_client->addToReceiveGroup(
		createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, _config.device_id),
		_haptic_device_switch_pressed);

	cout << "current position key: " << current_position_key << endl;
	Vector3d robot_position = _redis_client->getEigen(current_position_key);
	Matrix3d robot_orientation =
		_redis_client->getEigen(current_orientation_key);

	Affine3d robot_pose = Affine3d::Identity();
	robot_pose.translation() = robot_position;
	robot_pose.linear() = robot_orientation;

	_haptic_controller = make_unique<SaiPrimitives::HapticDeviceController>(
		device_limits, robot_pose, Affine3d::Identity(),
		_config.haptic_device_base_in_world.rotation());
	_haptic_output = _haptic_controller->getLatestOutput();

	// setup initial parametrization
	initialParametrizationFromConfig();

	// setup redis communication
	_redis_client->addToSendGroup(
		createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, _config.device_id),
		_haptic_output.device_command_force);
	_redis_client->addToSendGroup(
		createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, _config.device_id),
		_haptic_output.device_command_moment);
	_redis_client->addToSendGroup(robot_task_redis_key_prefix + "goal_position",
								  _haptic_output.robot_goal_position);
	_redis_client->addToSendGroup(
		robot_task_redis_key_prefix + "goal_orientation",
		_haptic_output.robot_goal_orientation);

	_redis_client->addToReceiveGroup(
		createRedisKey(POSITION_KEY_SUFFIX, _config.device_id),
		_haptic_input.device_position);
	_redis_client->addToReceiveGroup(
		createRedisKey(ROTATION_KEY_SUFFIX, _config.device_id),
		_haptic_input.device_orientation);
	_redis_client->addToReceiveGroup(
		createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, _config.device_id),
		_haptic_input.device_linear_velocity);
	_redis_client->addToReceiveGroup(
		createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, _config.device_id),
		_haptic_input.device_angular_velocity);
	_redis_client->addToReceiveGroup(
		robot_task_redis_key_prefix + "current_position",
		_haptic_input.robot_position);
	_redis_client->addToReceiveGroup(
		robot_task_redis_key_prefix + "current_orientation",
		_haptic_input.robot_orientation);
	_redis_client->addToReceiveGroup(
		robot_task_redis_key_prefix + "current_linear_velocity",
		_haptic_input.robot_linear_velocity);
	_redis_client->addToReceiveGroup(
		robot_task_redis_key_prefix + "current_angular_velocity",
		_haptic_input.robot_angular_velocity);
	_redis_client->addToReceiveGroup(
		robot_task_redis_key_prefix + "sensed_force",
		_haptic_input.robot_sensed_force);
	_redis_client->addToReceiveGroup(
		robot_task_redis_key_prefix + "sensed_moment",
		_haptic_input.robot_sensed_moment);

	const string haptic_controller_parameters_redis_key_prefix =
		"controllers::haptic_controllers::device" +
		std::to_string(_config.device_id) + "::parameters::";

	_redis_client->addToReceiveGroup(
		haptic_controller_parameters_redis_key_prefix +
			"plane_guidance::enabled",
		_config.plane_guidance_config.enabled);
	_redis_client->addToReceiveGroup(
		haptic_controller_parameters_redis_key_prefix +
			"line_guidance::enabled",
		_config.line_guidance_config.enabled);
	_redis_client->addToReceiveGroup(
		haptic_controller_parameters_redis_key_prefix +
			"workspace_virtual_limits::enabled",
		_config.impedance_mode_config.virtual_workspace_limits.enabled);
	_redis_client->addToReceiveGroup(
		haptic_controller_parameters_redis_key_prefix +
			"proxy_force_space_dimension",
		_config.impedance_mode_config.force_feedback
			.proxy_force_space_dimension);
	_redis_client->addToReceiveGroup(
		haptic_controller_parameters_redis_key_prefix + "proxy_force_axis",
		_config.impedance_mode_config.force_feedback.proxy_force_axis);
	_redis_client->addToReceiveGroup(
		haptic_controller_parameters_redis_key_prefix +
			"proxy_moment_space_dimension",
		_config.impedance_mode_config.force_feedback
			.proxy_moment_space_dimension);
	_redis_client->addToReceiveGroup(
		haptic_controller_parameters_redis_key_prefix + "proxy_moment_axis",
		_config.impedance_mode_config.force_feedback.proxy_moment_axis);

	// receive correct state from redis after waiting for a little
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	_redis_client->receiveAllFromGroup();

	// setup logging
	_logger = std::make_unique<SaiCommon::Logger>(
		_config.logger_config.folder_name + "/device" +
			std::to_string(_config.device_id) + "_control",
		_config.logger_config.add_timestamp_to_filename);

	_logger->addToLog(_haptic_input.device_position, "device_position");
	_logger->addToLog(_haptic_input.device_orientation, "device_orientation");
	_logger->addToLog(_haptic_input.device_linear_velocity,
					  "device_linear_velocity");
	_logger->addToLog(_haptic_input.device_angular_velocity,
					  "device_angular_velocity");
	_logger->addToLog(_haptic_input.robot_position, "robot_position");
	_logger->addToLog(_haptic_input.robot_orientation, "robot_orientation");
	_logger->addToLog(_haptic_input.robot_linear_velocity,
					  "robot_linear_velocity");
	_logger->addToLog(_haptic_input.robot_angular_velocity,
					  "robot_angular_velocity");
	_logger->addToLog(_haptic_input.robot_sensed_force, "robot_sensed_force");
	_logger->addToLog(_haptic_input.robot_sensed_moment, "robot_sensed_moment");
	_logger->addToLog(_haptic_output.device_command_force,
					  "device_command_force");
	_logger->addToLog(_haptic_output.device_command_moment,
					  "device_command_moment");
	_logger->addToLog(_haptic_output.robot_goal_position,
					  "robot_goal_position");
	_logger->addToLog(_haptic_output.robot_goal_orientation,
					  "robot_goal_orientation");

	_logger->addToLog(_config.plane_guidance_config.enabled,
					  "plane_guidance_enabled");
	_logger->addToLog(_config.plane_guidance_config.origin_point,
					  "plane_guidance_origin_point");
	_logger->addToLog(_config.plane_guidance_config.axis,
					  "plane_guidance_axis");
	_logger->addToLog(_config.line_guidance_config.enabled,
					  "line_guidance_enabled");
	_logger->addToLog(_config.line_guidance_config.origin_point,
					  "line_guidance_origin_point");
	_logger->addToLog(_config.line_guidance_config.axis, "line_guidance_axis");

	_logger->addToLog(
		_config.impedance_mode_config.virtual_workspace_limits.enabled,
		"workspace_virtual_limits_enabled");

	_logger->addToLog(_config.impedance_mode_config.force_feedback
						  .proxy_force_space_dimension,
					  "proxy_force_space_dimension");
	_logger->addToLog(
		_config.impedance_mode_config.force_feedback.proxy_force_axis,
		"proxy_force_axis");
	_logger->addToLog(_config.impedance_mode_config.force_feedback
						  .proxy_moment_space_dimension,
					  "proxy_moment_space_dimension");
	_logger->addToLog(
		_config.impedance_mode_config.force_feedback.proxy_moment_axis,
		"proxy_moment_axis");
}

void HapticDeviceControllerRedisInterface::runRedisCommunication(
	const std::atomic<bool>& user_stop_signal) {
	SaiCommon::LoopTimer timer(_config.control_frequency);
	timer.setTimerName(
		"HapticDeviceControllerRedisInterface Timer for redis "
		"communication on device: " +
		std::to_string(_config.device_id));

	while (!user_stop_signal && !external_stop_signal) {
		timer.waitForNextLoop();
		SaiPrimitives::HapticControllerOutput haptic_output;
		{
			lock_guard<mutex> lock(_haptic_output_mutex);
			haptic_output = _haptic_output;
		}

		_redis_client->sendAllFromGroup();
		_redis_client->receiveAllFromGroup();
	}
	timer.stop();
	// timer.printInfoPostRun();
}

void HapticDeviceControllerRedisInterface::initialParametrizationFromConfig() {
	_haptic_controller->setHapticControlType(
		SaiPrimitives::HapticControlType::HOMING);

	if (_config.orientation_teleop_enabled) {
		_haptic_controller->enableOrientationTeleop();
	} else {
		_haptic_controller->disableOrientationTeleop();
	}

	_haptic_controller->setHomingMaxVelocity(
		_config.homing_mode_config.max_linvel,
		_config.homing_mode_config.max_angvel);

	_haptic_controller->enableHapticWorkspaceVirtualLimits(
		_config.impedance_mode_config.virtual_workspace_limits.radius_limit,
		_config.impedance_mode_config.virtual_workspace_limits.angle_limit);
	if (!_config.impedance_mode_config.virtual_workspace_limits.enabled) {
		_haptic_controller->disableHapticWorkspaceVirtualLimits();
	}

	_haptic_controller->setScalingFactors(
		_config.impedance_mode_config.scaling_factors.scaling_factor_pos,
		_config.impedance_mode_config.scaling_factors.scaling_factor_ori);

	_haptic_controller->setVariableDampingGainsPos(
		_config.impedance_mode_config.variable_damping.linvel_thresholds,
		_config.impedance_mode_config.variable_damping.linear_damping);

	_haptic_controller->setVariableDampingGainsOri(
		_config.impedance_mode_config.variable_damping.angvel_thresholds,
		_config.impedance_mode_config.variable_damping.angular_damping);

	_haptic_controller->setReductionFactorForce(
		_config.impedance_mode_config.force_feedback.reduction_factor_force);
	_haptic_controller->setReductionFactorMoment(
		_config.impedance_mode_config.force_feedback.reduction_factor_moment);

	_haptic_controller->parametrizeProxyForceFeedbackSpace(
		_config.impedance_mode_config.force_feedback
			.proxy_force_space_dimension,
		_config.impedance_mode_config.force_feedback.proxy_force_axis);
	_haptic_controller->parametrizeProxyMomentFeedbackSpace(
		_config.impedance_mode_config.force_feedback
			.proxy_moment_space_dimension,
		_config.impedance_mode_config.force_feedback.proxy_moment_axis);

	_haptic_controller->setAdmittanceFactors(
		_config.admittance_mode_config.device_force_to_robot_delta_position,
		_config.admittance_mode_config
			.device_moment_to_robot_delta_orientation);
	_haptic_controller->setForceDeadbandForceMotionController(
		_config.admittance_mode_config.force_deadband);
	_haptic_controller->setMomentDeadbandForceMotionController(
		_config.admittance_mode_config.moment_deadband);

	_haptic_controller->enablePlaneGuidance(
		_config.plane_guidance_config.origin_point,
		_config.plane_guidance_config.axis);
	if (!_config.plane_guidance_config.enabled) {
		_haptic_controller->disablePlaneGuidance();
	}
	_haptic_controller->enableLineGuidance(
		_config.line_guidance_config.origin_point,
		_config.line_guidance_config.axis);
	if (!_config.line_guidance_config.enabled) {
		_haptic_controller->disableLineGuidance();
	}

	if (_config.control_gains_config.has_value()) {
		_haptic_controller->setDeviceControlGains(
			_config.control_gains_config->kp_pos,
			_config.control_gains_config->kv_pos,
			_config.control_gains_config->kp_ori,
			_config.control_gains_config->kv_ori);
	}

	if (_config.guidance_gains_config.has_value()) {
		_haptic_controller->setHapticGuidanceGains(
			_config.guidance_gains_config->kp_pos,
			_config.guidance_gains_config->kv_pos,
			_config.guidance_gains_config->kp_ori,
			_config.guidance_gains_config->kv_ori);
	}
}

void HapticDeviceControllerRedisInterface::processInputs() {
	// switch from homing to desired control type if homed
	if (_haptic_controller->getHapticControlType() ==
		SaiPrimitives::HapticControlType::HOMING) {
		if (_haptic_controller->getHomed() &&
				!_config.use_switch_to_exit_homing ||
			(!_haptic_device_switch_pressed &&
			 _haptic_device_switch_was_pressed)) {
			_haptic_controller->setHapticControlType(
				_config.control_mode ==
						HapticDeviceControllerConfig::ControlMode::IMPEDANCE
					? SaiPrimitives::HapticControlType::MOTION_MOTION
					: SaiPrimitives::HapticControlType::FORCE_MOTION);
		}
		_haptic_device_switch_was_pressed = _haptic_device_switch_pressed;
		return;
	}

	// haptic device switch function
	bool switch_behavior_attached_to_haptic_button_press = false;
	if (_config.switch_usage_type ==
		HapticDeviceControllerConfig::SwitchUsageType::CLICK) {
		switch_behavior_attached_to_haptic_button_press =
			_haptic_device_switch_pressed && !_haptic_device_switch_was_pressed;
	} else if (_config.switch_usage_type ==
			   HapticDeviceControllerConfig::SwitchUsageType::HOLD) {
		switch_behavior_attached_to_haptic_button_press =
			_haptic_device_switch_pressed != _haptic_device_switch_was_pressed;
	}

	switch (_config.switch_function) {
		case HapticDeviceControllerConfig::SwitchFunction::CLUTCH:
			if (switch_behavior_attached_to_haptic_button_press) {
				if (_haptic_controller->getHapticControlType() ==
					SaiPrimitives::HapticControlType::MOTION_MOTION) {
					_haptic_controller->setHapticControlType(
						SaiPrimitives::HapticControlType::CLUTCH);
				} else if (_config.control_mode ==
							   HapticDeviceControllerConfig::ControlMode::
								   IMPEDANCE &&
						   _haptic_controller->getHapticControlType() ==
							   SaiPrimitives::HapticControlType::CLUTCH) {
					_haptic_controller->setHapticControlType(
						SaiPrimitives::HapticControlType::MOTION_MOTION);
				}
			}
			break;
		case HapticDeviceControllerConfig::SwitchFunction::ORIENTATION_CONTROL:
			if (switch_behavior_attached_to_haptic_button_press) {
				if (_haptic_controller->getOrientationTeleopEnabled()) {
					_haptic_controller->disableOrientationTeleop();
				} else {
					_haptic_controller->enableOrientationTeleop();
				}
			}
			break;
		default:
			break;
	}
	_haptic_device_switch_was_pressed = _haptic_device_switch_pressed;

	// proxy space parametrization
	_haptic_controller->parametrizeProxyForceFeedbackSpace(
		_config.impedance_mode_config.force_feedback
			.proxy_force_space_dimension,
		_config.impedance_mode_config.force_feedback.proxy_force_axis);
	_haptic_controller->parametrizeProxyMomentFeedbackSpace(
		_config.impedance_mode_config.force_feedback
			.proxy_moment_space_dimension,
		_config.impedance_mode_config.force_feedback.proxy_moment_axis);

	// plane guidance
	if (_config.plane_guidance_config.enabled &&
		!_haptic_controller->getPlaneGuidanceEnabled()) {
		if (_config.plane_guidance_config.set_origin_to_current_position) {
			_config.plane_guidance_config.origin_point =
				_haptic_input.device_position;
		}
		_haptic_controller->enablePlaneGuidance(
			_config.plane_guidance_config.origin_point,
			_config.plane_guidance_config.axis);
	} else if (!_config.plane_guidance_config.enabled &&
			   _haptic_controller->getPlaneGuidanceEnabled()) {
		_haptic_controller->disablePlaneGuidance();
	}

	// line guidance
	if (_config.line_guidance_config.enabled &&
		!_haptic_controller->getLineGuidanceEnabled()) {
		if (_config.line_guidance_config.set_origin_to_current_position) {
			_config.line_guidance_config.origin_point =
				_haptic_input.device_position;
		}
		_haptic_controller->enableLineGuidance(
			_config.line_guidance_config.origin_point,
			_config.line_guidance_config.axis);
	} else if (!_config.line_guidance_config.enabled &&
			   _haptic_controller->getLineGuidanceEnabled()) {
		_haptic_controller->disableLineGuidance();
	}

	// workspace virtual limits
	if (_config.impedance_mode_config.virtual_workspace_limits.enabled &&
		!_haptic_controller->getHapticWorkspaceVirtualLimitsEnabled()) {
		_haptic_controller->enableHapticWorkspaceVirtualLimits(
			_config.impedance_mode_config.virtual_workspace_limits.radius_limit,
			_config.impedance_mode_config.virtual_workspace_limits.angle_limit);
	} else if (!_config.impedance_mode_config.virtual_workspace_limits
					.enabled &&
			   _haptic_controller->getHapticWorkspaceVirtualLimitsEnabled()) {
		_haptic_controller->disableHapticWorkspaceVirtualLimits();
	}

	// logging
	switch (_logging_state) {
		case LoggingState::OFF:
			if (_logging_on) {
				_logging_state = LoggingState::START;
			}
			break;
		case LoggingState::START:
			_logger->start(_config.logger_config.frequency);
			_logging_state = LoggingState::ON;
			break;

		case LoggingState::ON:
			if (!_logging_on) {
				_logging_state = LoggingState::STOP;
			}
			break;

		case LoggingState::STOP:
			_logger->stop();
			_logging_state = LoggingState::OFF;
			break;

		default:
			break;
	}
}

}  // namespace SaiInterfaces