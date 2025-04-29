#include "RobotControllerRedisInterface.h"

#include <signal.h>

#include <filesystem>

using namespace std;
using namespace Eigen;

namespace SaiInterfaces {

namespace {

bool external_stop_signal = false;
void stop(int i) { external_stop_signal = true; }

const std::string reset_inputs_redis_group = "reset_input_group";

}  // namespace

RobotControllerRedisInterface::RobotControllerRedisInterface(
	const RobotControllerConfig& config, const bool setup_signal_handler) {
	_config = config;

	_logging_on = _config.logger_config.start_with_logger_on;
	_logging_state = _logging_on ? LoggingState::START : LoggingState::OFF;

	_redis_client = make_unique<SaiCommon::RedisClient>(
		_config.redis_config.redis_namespace_prefix);
	_redis_client->connect(_config.redis_config.redis_ip,
						   _config.redis_config.redis_port);

	_reset_redis_inputs = false;
	initialize();

	if (setup_signal_handler) {
		signal(SIGABRT, &stop);
		signal(SIGTERM, &stop);
		signal(SIGINT, &stop);
	}
}

void RobotControllerRedisInterface::runRedisCommunication(
	const std::atomic<bool>& user_stop_signal) {
	SaiCommon::LoopTimer timer(_config.control_frequency);
	timer.setTimerName(
		"RobotControllerRedisInterface Timer for redis communication on "
		"robot: " +
		_config.robot_name);

	while (!user_stop_signal && !external_stop_signal) {
		timer.waitForNextLoop();
		{
			lock_guard<mutex> lock(_control_torques_mutex);
			// WARNING: need to use the assignment operator << here to keep the
			// same memory address for the redis communication that references
			// this by pointer
			_robot_command_torques << _robot_command_torques_local;
		}
		{
			lock_guard<mutex> lock(_switching_controller_mutex);

			if (_reset_redis_inputs) {
				_redis_client->sendAllFromGroup(
					{"default", reset_inputs_redis_group});
				_reset_redis_inputs = false;
			} else {
				_redis_client->sendAllFromGroup();
			}

			_redis_client->receiveAllFromGroup(
				std::vector<std::string>{"default", _active_controller_name});
			_robot_q_local = _robot_q;
			_robot_dq_local = _robot_dq;
			if (_config.get_mass_matrix_from_redis) {
				std::lock_guard<std::mutex> lock(_mass_matrix_mutex);
				_robot_M_local = _robot_M;
			}
		}
	}
	timer.stop();
	// timer.printInfoPostRun();

	// let redis know the controller is no longer running for that robot
	_redis_client->setBool(
		"controllers::" + _config.robot_name + "::is_running", false);
}

void RobotControllerRedisInterface::run(
	const std::atomic<bool>& user_stop_signal) {
	// start redis communication thread
	std::thread redis_communication_thread(
		&RobotControllerRedisInterface::runRedisCommunication, this,
		std::ref(user_stop_signal));

	// create timer
	SaiCommon::LoopTimer timer(_config.control_frequency);
	timer.setTimerName(
		"RobotControllerRedisInterface Timer for controller on robot: " +
		_config.robot_name);

	while (!user_stop_signal && !external_stop_signal) {
		timer.waitForNextLoop();

		// switch controller if needed
		switchController(_tentative_next_active_controller_name);

		if (_reset_redis_inputs) {
			// let the inputs reset and keep the same control for one tick
			// if the reset was triggered by a controller switching
			continue;
		}

		// process inputs
		processInputs();

		if (_reset_redis_inputs) {
			// let the inputs reset and keep the same control for one tick
			// repeat here in case the reset inputs was triggered by a config
			// change in process inputs
			continue;
		}

		// update task models
		_robot_model->setQ(_robot_q_local);
		_robot_model->setDq(_robot_dq_local);
		if (_config.get_mass_matrix_from_redis) {
			std::lock_guard<std::mutex> lock(_mass_matrix_mutex);
			_robot_model->updateModel(_robot_M_local);
		} else {
			_robot_model->updateModel();
			_robot_M_local = _robot_model->M();
		}
		_robot_controllers.at(_active_controller_name)
			->updateControllerTaskModels();

		// compute torques
		{
			lock_guard<mutex> lock(_control_torques_mutex);
			_robot_command_torques_local =
				_robot_controllers.at(_active_controller_name)
					->computeControlTorques();
		}
	}
	timer.stop();

	// stop logging
	_robot_logger->stop();
	for (auto& task_loggers : _task_loggers) {
		for (auto& pair : task_loggers.second) {
			pair.second->stop();
		}
	}

	// stop redis communication
	redis_communication_thread.join();

	_redis_client->setEigen(
		"commands::" + _config.robot_name + "::control_torques",
		Eigen::VectorXd::Zero(_robot_model->dof()));

	timer.printInfoPostRun();
}

void RobotControllerRedisInterface::initialize() {
	_robot_model = make_shared<SaiModel::SaiModel>(_config.robot_model_file);
	_robot_model->setTRobotBase(_config.robot_base_in_world);
	_robot_model->setWorldGravity(_config.world_gravity);

	std::string joint_names_str = "[";
	for (const auto& joint_name : _robot_model->jointNames()) {
		joint_names_str += joint_name + ",";
	}
	joint_names_str.pop_back();
	joint_names_str += "]";

	_redis_client->set("controllers::" + _config.robot_name + "::joint_names",
					   joint_names_str);

	_robot_q.setZero(_robot_model->dof());
	_robot_q_local.setZero(_robot_model->dof());
	_robot_dq.setZero(_robot_model->dof());
	_robot_dq_local.setZero(_robot_model->dof());
	_robot_command_torques.setZero(_robot_model->dof());
	_robot_command_torques_local.setZero(_robot_model->dof());
	_robot_M.setIdentity(_robot_model->dof(), _robot_model->dof());
	_robot_M_local.setIdentity(_robot_model->dof(), _robot_model->dof());

	_redis_client->addToSendGroup(
		"commands::" + _config.robot_name + "::control_torques",
		_robot_command_torques);

	_redis_client->addToReceiveGroup(
		"sensors::" + _config.robot_name + "::joint_positions", _robot_q);
	_redis_client->addToReceiveGroup(
		"sensors::" + _config.robot_name + "::joint_velocities", _robot_dq);
	if (_config.get_mass_matrix_from_redis) {
		_redis_client->addToReceiveGroup(
			"sensors::" + _config.robot_name + "::model::mass_matrix",
			_robot_M);
	}
	_redis_client->addToReceiveGroup(
		"controllers::" + _config.robot_name + "::logging_on", _logging_on);

	// let redis know the controller is running for that robot
	_redis_client->setBool(
		"controllers::" + _config.robot_name + "::is_running", true);

	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	_redis_client->receiveAllFromGroup();
	_robot_q_local = _robot_q;
	_robot_model->setQ(_robot_q_local);
	if (_config.get_mass_matrix_from_redis) {
		std::lock_guard<std::mutex> lock(_mass_matrix_mutex);
		_robot_model->updateModel(_robot_M_local);
	} else {
		_robot_model->updateModel();
		_robot_M_local = _robot_model->M();
	}

	for (const auto& pair : _config.single_controller_configs) {
		vector<shared_ptr<SaiPrimitives::TemplateTask>> ordered_tasks_list;

		_is_active_controller[pair.first] = false;

		for (const auto& tasks : pair.second.tasks_configs) {
			if (holds_alternative<JointTaskConfig>(tasks)) {
				const auto& joint_task_config = get<JointTaskConfig>(tasks);

				// selection matrix for controlled joints
				MatrixXd S = MatrixXd::Identity(_robot_model->dof(),
												_robot_model->dof());

				if (!joint_task_config.controlled_joint_names.empty()) {
					S.setZero(joint_task_config.controlled_joint_names.size(),
							  _robot_model->dof());
					for (int i = 0;
						 i < joint_task_config.controlled_joint_names.size();
						 i++) {
						S(i, _robot_model->jointIndex(
								 joint_task_config.controlled_joint_names[i])) =
							1;
					}
				}

				// create joint task
				auto joint_task = make_shared<SaiPrimitives::JointTask>(
					_robot_model, S, joint_task_config.task_name,
					1.0 / _config.control_frequency);

				ordered_tasks_list.push_back(joint_task);

			} else if (holds_alternative<MotionForceTaskConfig>(tasks)) {
				const auto& motion_force_task_config =
					get<MotionForceTaskConfig>(tasks);

				// create task
				auto motion_force_task =
					make_shared<SaiPrimitives::MotionForceTask>(
						_robot_model, motion_force_task_config.link_name,
						motion_force_task_config.controlled_directions_position,
						motion_force_task_config
							.controlled_directions_orientation,
						motion_force_task_config.compliant_frame,
						motion_force_task_config.task_name,
						motion_force_task_config
							.is_parametrization_in_compliant_frame,
						1.0 / _config.control_frequency);

				ordered_tasks_list.push_back(motion_force_task);
			}
		}
		_robot_controllers[pair.first] =
			make_unique<SaiPrimitives::RobotController>(_robot_model,
														ordered_tasks_list);
		_robot_controllers.at(pair.first)
			->enableGravityCompensation(
				pair.second.enable_gravity_compensation);
		_robot_controllers.at(pair.first)
			->enableJointLimitAvoidance(
				pair.second.enable_joint_limit_avoidance);
		_robot_controllers.at(pair.first)
			->enableTorqueSaturation(pair.second.enable_torque_saturation);
	}

	if (_robot_controllers.find(_config.initial_active_controller_name) ==
		_robot_controllers.end()) {
		throw std::runtime_error(
			"Initial active controller name does not match any controller in "
			"the config file");
	}

	_tentative_next_active_controller_name =
		_config.initial_active_controller_name;
	_redis_client->addToReceiveGroup(
		"controllers::" + _config.robot_name + "::active_controller_name",
		_tentative_next_active_controller_name);
	initializeRedisTasksIO();

	switchController(_tentative_next_active_controller_name);
}

void RobotControllerRedisInterface::switchController(
	const std::string& controller_name) {
	if (controller_name == _active_controller_name) {
		return;
	}
	if (_robot_controllers.find(controller_name) == _robot_controllers.end()) {
		cout << "WARNING: Controller name does not match any controller in the "
				"config file"
			 << endl;
		return;
	}
	{
		lock_guard<mutex> lock(_switching_controller_mutex);
		if (_active_controller_name != "") {  // for the initialization case
			_is_active_controller.at(_active_controller_name) = false;
		}
		_active_controller_name = controller_name;
		_is_active_controller.at(_active_controller_name) = true;

		_robot_model->setQ(_robot_q_local);
		_robot_model->setDq(_robot_dq_local);
		if (_config.get_mass_matrix_from_redis) {
			std::lock_guard<std::mutex> lock(_mass_matrix_mutex);
			_robot_model->updateModel(_robot_M_local);
		} else {
			_robot_model->updateModel();
			_robot_M_local = _robot_model->M();
		}
		_robot_controllers.at(_active_controller_name)->reinitializeTasks();

		// reset inputs for new active controller
		for (auto& task_input :
			 _controller_inputs.at(_active_controller_name)) {
			if (holds_alternative<JointTaskInput>(task_input.second)) {
				auto& joint_task_input = get<JointTaskInput>(task_input.second);
				joint_task_input.setFromTask(
					_robot_controllers.at(_active_controller_name)
						->getJointTaskByName(task_input.first));
			} else if (holds_alternative<MotionForceTaskInput>(
						   task_input.second)) {
				auto& motion_force_task_input =
					get<MotionForceTaskInput>(task_input.second);
				motion_force_task_input.setFromTask(
					_robot_controllers.at(_active_controller_name)
						->getMotionForceTaskByName(task_input.first));
			}
		}
		_reset_redis_inputs = true;
	}
}

void RobotControllerRedisInterface::initializeRedisTasksIO() {
	_redis_client->createNewSendGroup(reset_inputs_redis_group);

	if (!std::filesystem::exists(_config.logger_config.folder_name)) {
		std::filesystem::create_directories(_config.logger_config.folder_name);
	}

	_robot_logger = std::make_unique<SaiCommon::Logger>(
		_config.logger_config.folder_name + '/' + _config.robot_name +
			"_control",
		_config.logger_config.add_timestamp_to_filename);

	_robot_logger->addToLog(_robot_q_local, "joint_positions");
	_robot_logger->addToLog(_robot_dq_local, "joint_velocities");
	_robot_logger->addToLog(_robot_command_torques_local, "control_torques");
	_robot_logger->addToLog(_robot_M_local, "mass_matrix");

	for (auto& pair : _config.single_controller_configs) {
		const string& controller_name = pair.first;

		_controller_inputs[controller_name] = {};
		_controller_task_monitoring_data[controller_name] = {};
		_redis_client->createNewReceiveGroup(controller_name);

		const std::string& current_controller_logger_folder =
			_config.logger_config.folder_name + '/' + _config.robot_name + '_' +
			controller_name;
		if (!std::filesystem::exists(current_controller_logger_folder)) {
			std::filesystem::create_directory(current_controller_logger_folder);
		}

		auto& task_configs = pair.second.tasks_configs;

		for (auto& task_config : task_configs) {
			if (holds_alternative<JointTaskConfig>(task_config)) {
				auto& joint_task_config = get<JointTaskConfig>(task_config);
				const string task_name = joint_task_config.task_name;
				auto joint_task = _robot_controllers.at(controller_name)
									  ->getJointTaskByName(task_name);
				_controller_inputs.at(controller_name)[task_name] =
					JointTaskInput(joint_task->getTaskDof());
				_controller_task_monitoring_data.at(
					controller_name)[task_name] =
					JointTaskMonitoringData(joint_task->getTaskDof());

				_task_loggers[controller_name][task_name] =
					std::make_unique<SaiCommon::Logger>(
						current_controller_logger_folder + '/' + task_name,
						_config.logger_config.add_timestamp_to_filename);
				auto task_logger =
					_task_loggers.at(controller_name).at(task_name).get();
				task_logger->addToLog(_is_active_controller.at(controller_name),
									  "is_active");

				const string& key_prefix =
					"controllers::" + _config.robot_name +
					"::" + controller_name + "::" + task_name + "::";

				// dynamic decoupling
				task_logger->addToLog(joint_task_config.use_dynamic_decoupling,
									  "use_dynamic_decoupling");
				_redis_client->addToReceiveGroup(
					key_prefix + "use_dynamic_decoupling",
					joint_task_config.use_dynamic_decoupling, controller_name);

				// gains
				task_logger->addToLog(joint_task_config.gains_config.kp, "kp");
				task_logger->addToLog(joint_task_config.gains_config.kv, "kv");
				task_logger->addToLog(joint_task_config.gains_config.ki, "ki");
				_redis_client->addToReceiveGroup(
					key_prefix + "kp", joint_task_config.gains_config.kp,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "kv", joint_task_config.gains_config.kv,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "ki", joint_task_config.gains_config.ki,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "gains_safety_checks_enabled",
					joint_task_config.gains_config.safety_checks_enabled,
					controller_name);

				// velocity saturation
				task_logger->addToLog(
					joint_task_config.velocity_saturation_config.enabled,
					"velocity_saturation_enabled");
				task_logger->addToLog(
					joint_task_config.velocity_saturation_config.velocity_limit,
					"velocity_saturation_limit");
				_redis_client->addToReceiveGroup(
					key_prefix + "velocity_saturation_enabled",
					joint_task_config.velocity_saturation_config.enabled,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "velocity_saturation_limit",
					joint_task_config.velocity_saturation_config.velocity_limit,
					controller_name);

				// otg
				task_logger->addToLog(joint_task_config.otg_config.enabled,
									  "otg_enabled");
				task_logger->addToLog(joint_task_config.otg_config.jerk_limited,
									  "otg_jerk_limited");
				task_logger->addToLog(
					joint_task_config.otg_config.limits.max_velocity,
					"otg_max_velocity");
				task_logger->addToLog(
					joint_task_config.otg_config.limits.max_acceleration,
					"otg_max_acceleration");
				task_logger->addToLog(
					joint_task_config.otg_config.limits.max_jerk,
					"otg_max_jerk");
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_enabled",
					joint_task_config.otg_config.enabled, controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_jerk_limited",
					joint_task_config.otg_config.jerk_limited, controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_velocity",
					joint_task_config.otg_config.limits.max_velocity,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_acceleration",
					joint_task_config.otg_config.limits.max_acceleration,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_jerk",
					joint_task_config.otg_config.limits.max_jerk,
					controller_name);

				// inputs
				JointTaskInput& joint_task_input = std::get<JointTaskInput>(
					_controller_inputs.at(controller_name).at(task_name));
				joint_task_input.setFromTask(joint_task);
				task_logger->addToLog(joint_task_input.goal_position,
									  "goal_position");
				task_logger->addToLog(joint_task_input.goal_velocity,
									  "goal_velocity");
				task_logger->addToLog(joint_task_input.goal_acceleration,
									  "goal_acceleration");
				_redis_client->addToSendGroup(key_prefix + "goal_position",
											  joint_task_input.goal_position,
											  reset_inputs_redis_group);
				_redis_client->addToSendGroup(key_prefix + "goal_velocity",
											  joint_task_input.goal_velocity,
											  reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "goal_acceleration",
					joint_task_input.goal_acceleration,
					reset_inputs_redis_group);
				_redis_client->addToReceiveGroup(key_prefix + "goal_position",
												 joint_task_input.goal_position,
												 controller_name);
				_redis_client->addToReceiveGroup(key_prefix + "goal_velocity",
												 joint_task_input.goal_velocity,
												 controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "goal_acceleration",
					joint_task_input.goal_acceleration, controller_name);

				// monitoring data
				JointTaskMonitoringData& joint_task_monitoring_data =
					std::get<JointTaskMonitoringData>(
						_controller_task_monitoring_data.at(controller_name)
							.at(task_name));
				task_logger->addToLog(
					joint_task_monitoring_data.current_position,
					"current_position");
				task_logger->addToLog(
					joint_task_monitoring_data.current_velocity,
					"current_velocity");
				task_logger->addToLog(
					joint_task_monitoring_data.desired_position,
					"desired_position");
				task_logger->addToLog(
					joint_task_monitoring_data.desired_velocity,
					"desired_velocity");
				task_logger->addToLog(
					joint_task_monitoring_data.desired_acceleration,
					"desired_acceleration");
				_redis_client->addToSendGroup(
					key_prefix + "current_position",
					joint_task_monitoring_data.current_position);
				_redis_client->addToSendGroup(
					key_prefix + "current_velocity",
					joint_task_monitoring_data.current_velocity);

			} else if (holds_alternative<MotionForceTaskConfig>(task_config)) {
				auto& motion_force_task_config =
					get<MotionForceTaskConfig>(task_config);
				const string task_name = motion_force_task_config.task_name;

				auto motion_force_task =
					_robot_controllers.at(controller_name)
						->getMotionForceTaskByName(task_name);

				_controller_inputs.at(controller_name)[task_name] =
					MotionForceTaskInput();
				_controller_task_monitoring_data.at(
					controller_name)[task_name] =
					MotionForceTaskMonitoringData();

				_task_loggers[controller_name][task_name] =
					std::make_unique<SaiCommon::Logger>(
						current_controller_logger_folder + '/' + task_name,
						_config.logger_config.add_timestamp_to_filename);
				auto task_logger =
					_task_loggers.at(controller_name).at(task_name).get();
				task_logger->addToLog(_is_active_controller.at(controller_name),
									  "is_active");

				const string key_prefix = "controllers::" + _config.robot_name +
										  "::" + controller_name +
										  "::" + task_name + "::";

				// dynamic decoupling
				task_logger->addToLog(
					motion_force_task_config.use_dynamic_decoupling,
					"use_dynamic_decoupling");
				_redis_client->addToReceiveGroup(
					key_prefix + "use_dynamic_decoupling",
					motion_force_task_config.use_dynamic_decoupling,
					controller_name);

				// force control parametrization
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.force_space_param_config.force_space_dimension,
					"force_space_dimension");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.force_space_param_config.axis,
					"force_or_motion_axis");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.moment_space_param_config.force_space_dimension,
					"moment_space_dimension");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.moment_space_param_config.axis,
					"moment_or_rotmotion_axis");
				_redis_client->addToReceiveGroup(
					key_prefix + "closed_loop_force_control",
					motion_force_task_config.force_control_config
						.closed_loop_force_control,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "force_space_dimension",
					motion_force_task_config.force_control_config
						.force_space_param_config.force_space_dimension,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "force_space_axis",
					motion_force_task_config.force_control_config
						.force_space_param_config.axis,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "moment_space_dimension",
					motion_force_task_config.force_control_config
						.moment_space_param_config.force_space_dimension,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "moment_space_axis",
					motion_force_task_config.force_control_config
						.moment_space_param_config.axis,
					controller_name);

				// velocity saturation
				task_logger->addToLog(
					motion_force_task_config.velocity_saturation_config.enabled,
					"velocity_saturation_enabled");
				task_logger->addToLog(
					motion_force_task_config.velocity_saturation_config
						.linear_velocity_limit,
					"linear_velocity_saturation_limit");
				task_logger->addToLog(
					motion_force_task_config.velocity_saturation_config
						.angular_velocity_limit,
					"angular_velocity_saturation_limit");
				_redis_client->addToReceiveGroup(
					key_prefix + "velocity_saturation_enabled",
					motion_force_task_config.velocity_saturation_config.enabled,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "linear_velocity_saturation_limit",
					motion_force_task_config.velocity_saturation_config
						.linear_velocity_limit,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "angular_velocity_saturation_limit",
					motion_force_task_config.velocity_saturation_config
						.angular_velocity_limit,
					controller_name);

				// otg
				task_logger->addToLog(
					motion_force_task_config.otg_config.enabled, "otg_enabled");
				task_logger->addToLog(
					motion_force_task_config.otg_config.jerk_limited,
					"otg_jerk_limited");
				task_logger->addToLog(
					motion_force_task_config.otg_config.max_linear_velocity,
					"otg_max_linear_velocity");
				task_logger->addToLog(
					motion_force_task_config.otg_config.max_angular_velocity,
					"otg_max_angular_velocity");
				task_logger->addToLog(
					motion_force_task_config.otg_config.max_linear_acceleration,
					"otg_max_linear_acceleration");
				task_logger->addToLog(motion_force_task_config.otg_config
										  .max_angular_acceleration,
									  "otg_max_angular_acceleration");
				task_logger->addToLog(
					motion_force_task_config.otg_config.max_linear_jerk,
					"otg_max_linear_jerk");
				task_logger->addToLog(
					motion_force_task_config.otg_config.max_angular_jerk,
					"otg_max_angular_jerk");
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_enabled",
					motion_force_task_config.otg_config.enabled,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_jerk_limited",
					motion_force_task_config.otg_config.jerk_limited,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_linear_velocity",
					motion_force_task_config.otg_config.max_linear_velocity,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_angular_velocity",
					motion_force_task_config.otg_config.max_angular_velocity,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_linear_acceleration",
					motion_force_task_config.otg_config.max_linear_acceleration,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_angular_acceleration",
					motion_force_task_config.otg_config
						.max_angular_acceleration,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_linear_jerk",
					motion_force_task_config.otg_config.max_linear_jerk,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "otg_max_angular_jerk",
					motion_force_task_config.otg_config.max_angular_jerk,
					controller_name);

				// gains
				task_logger->addToLog(
					motion_force_task_config.position_gains_config.kp,
					"position_kp");
				task_logger->addToLog(
					motion_force_task_config.position_gains_config.kv,
					"position_kv");
				task_logger->addToLog(
					motion_force_task_config.position_gains_config.ki,
					"position_ki");
				task_logger->addToLog(
					motion_force_task_config.orientation_gains_config.kp,
					"orientation_kp");
				task_logger->addToLog(
					motion_force_task_config.orientation_gains_config.kv,
					"orientation_kv");
				task_logger->addToLog(
					motion_force_task_config.orientation_gains_config.ki,
					"orientation_ki");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.force_gains_config.kp,
					"force_kp");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.force_gains_config.kv,
					"force_kv");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.force_gains_config.ki,
					"force_ki");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.moment_gains_config.kp,
					"moment_kp");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.moment_gains_config.kv,
					"moment_kv");
				task_logger->addToLog(
					motion_force_task_config.force_control_config
						.moment_gains_config.ki,
					"moment_ki");
				_redis_client->addToReceiveGroup(
					key_prefix + "position_kp",
					motion_force_task_config.position_gains_config.kp,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "position_kv",
					motion_force_task_config.position_gains_config.kv,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "position_ki",
					motion_force_task_config.position_gains_config.ki,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "position_gains_safety_checks_enabled",
					motion_force_task_config.position_gains_config
						.safety_checks_enabled,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "orientation_kp",
					motion_force_task_config.orientation_gains_config.kp,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "orientation_kv",
					motion_force_task_config.orientation_gains_config.kv,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "orientation_ki",
					motion_force_task_config.orientation_gains_config.ki,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "orientation_gains_safety_checks_enabled",
					motion_force_task_config.orientation_gains_config
						.safety_checks_enabled,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "force_kp",
					motion_force_task_config.force_control_config
						.force_gains_config.kp,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "force_kv",
					motion_force_task_config.force_control_config
						.force_gains_config.kv,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "force_ki",
					motion_force_task_config.force_control_config
						.force_gains_config.ki,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "moment_kp",
					motion_force_task_config.force_control_config
						.moment_gains_config.kp,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "moment_kv",
					motion_force_task_config.force_control_config
						.moment_gains_config.kv,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "moment_ki",
					motion_force_task_config.force_control_config
						.moment_gains_config.ki,
					controller_name);

				// input
				MotionForceTaskInput& motion_force_task_input =
					std::get<MotionForceTaskInput>(
						_controller_inputs.at(controller_name).at(task_name));
				motion_force_task_input.setFromTask(motion_force_task);
				task_logger->addToLog(motion_force_task_input.goal_position,
									  "goal_position");
				task_logger->addToLog(
					motion_force_task_input.goal_linear_velocity,
					"goal_linear_velocity");
				task_logger->addToLog(
					motion_force_task_input.goal_linear_acceleration,
					"goal_linear_acceleration");
				task_logger->addToLog(motion_force_task_input.goal_orientation,
									  "goal_orientation");
				task_logger->addToLog(
					motion_force_task_input.goal_angular_velocity,
					"goal_angular_velocity");
				task_logger->addToLog(
					motion_force_task_input.goal_angular_acceleration,
					"goal_angular_acceleration");
				task_logger->addToLog(motion_force_task_input.desired_force,
									  "desired_force");
				task_logger->addToLog(motion_force_task_input.desired_moment,
									  "desired_moment");
				task_logger->addToLog(
					motion_force_task_input.sensed_force_sensor_frame,
					"sensed_force_sensor_frame");
				task_logger->addToLog(
					motion_force_task_input.sensed_moment_sensor_frame,
					"sensed_moment_sensor_frame");
				_redis_client->addToSendGroup(
					key_prefix + "goal_position",
					motion_force_task_input.goal_position,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "goal_linear_velocity",
					motion_force_task_input.goal_linear_velocity,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "goal_linear_acceleration",
					motion_force_task_input.goal_linear_acceleration,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "goal_orientation",
					motion_force_task_input.goal_orientation,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "goal_angular_velocity",
					motion_force_task_input.goal_angular_velocity,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "goal_angular_acceleration",
					motion_force_task_input.goal_angular_acceleration,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "desired_force",
					motion_force_task_input.desired_force,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					key_prefix + "desired_moment",
					motion_force_task_input.desired_moment,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					"sensors::" + _config.robot_name + "::ft_sensor::" +
						motion_force_task_config.link_name + "::force",
					motion_force_task_input.sensed_force_sensor_frame,
					reset_inputs_redis_group);
				_redis_client->addToSendGroup(
					"sensors::" + _config.robot_name + "::ft_sensor::" +
						motion_force_task_config.link_name + "::moment",
					motion_force_task_input.sensed_moment_sensor_frame,
					reset_inputs_redis_group);
				_redis_client->addToReceiveGroup(
					key_prefix + "goal_position",
					motion_force_task_input.goal_position, controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "goal_linear_velocity",
					motion_force_task_input.goal_linear_velocity,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "goal_linear_acceleration",
					motion_force_task_input.goal_linear_acceleration,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "goal_orientation",
					motion_force_task_input.goal_orientation, controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "goal_angular_velocity",
					motion_force_task_input.goal_angular_velocity,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "goal_angular_acceleration",
					motion_force_task_input.goal_angular_acceleration,
					controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "desired_force",
					motion_force_task_input.desired_force, controller_name);
				_redis_client->addToReceiveGroup(
					key_prefix + "desired_moment",
					motion_force_task_input.desired_moment, controller_name);
				_redis_client->addToReceiveGroup(
					"sensors::" + _config.robot_name + "::ft_sensor::" +
						motion_force_task_config.link_name + "::force",
					motion_force_task_input.sensed_force_sensor_frame,
					controller_name);
				_redis_client->addToReceiveGroup(
					"sensors::" + _config.robot_name + "::ft_sensor::" +
						motion_force_task_config.link_name + "::moment",
					motion_force_task_input.sensed_moment_sensor_frame,
					controller_name);

				// monitoring data
				MotionForceTaskMonitoringData&
					motion_force_task_monitoring_data =
						std::get<MotionForceTaskMonitoringData>(
							_controller_task_monitoring_data.at(controller_name)
								.at(task_name));
				task_logger->addToLog(
					motion_force_task_monitoring_data.current_position,
					"current_position");
				task_logger->addToLog(
					motion_force_task_monitoring_data.current_orientation,
					"current_orientation");
				task_logger->addToLog(
					motion_force_task_monitoring_data.current_linear_velocity,
					"current_linear_velocity");
				task_logger->addToLog(
					motion_force_task_monitoring_data.current_angular_velocity,
					"current_angular_velocity");
				task_logger->addToLog(
					motion_force_task_monitoring_data.desired_position,
					"desired_position");
				task_logger->addToLog(
					motion_force_task_monitoring_data.desired_orientation,
					"desired_orientation");
				task_logger->addToLog(
					motion_force_task_monitoring_data.desired_linear_velocity,
					"desired_linear_velocity");
				task_logger->addToLog(
					motion_force_task_monitoring_data.desired_angular_velocity,
					"desired_angular_velocity");
				task_logger->addToLog(motion_force_task_monitoring_data
										  .desired_linear_acceleration,
									  "desired_linear_acceleration");
				task_logger->addToLog(motion_force_task_monitoring_data
										  .desired_angular_acceleration,
									  "desired_angular_acceleration");
				task_logger->addToLog(
					motion_force_task_monitoring_data.sensed_force_world_frame,
					"sensed_force_world_frame");
				task_logger->addToLog(
					motion_force_task_monitoring_data.sensed_moment_world_frame,
					"sensed_moment_world_frame");
				_redis_client->addToSendGroup(
					key_prefix + "sensed_force",
					motion_force_task_monitoring_data.sensed_force_world_frame);
				_redis_client->addToSendGroup(key_prefix + "sensed_moment",
											  motion_force_task_monitoring_data
												  .sensed_moment_world_frame);
				_redis_client->addToSendGroup(
					key_prefix + "current_position",
					motion_force_task_monitoring_data.current_position);
				_redis_client->addToSendGroup(
					key_prefix + "current_orientation",
					motion_force_task_monitoring_data.current_orientation);
				_redis_client->addToSendGroup(
					key_prefix + "current_linear_velocity",
					motion_force_task_monitoring_data.current_linear_velocity);
				_redis_client->addToSendGroup(
					key_prefix + "current_angular_velocity",
					motion_force_task_monitoring_data.current_angular_velocity);
			}
		}
	}
}

void RobotControllerRedisInterface::processInputs() {
	bool reset_inputs = false;

	auto& current_controller_config =
		_config.single_controller_configs.at(_active_controller_name);

	for (auto& task_config : current_controller_config.tasks_configs) {
		if (holds_alternative<JointTaskConfig>(task_config)) {
			auto& joint_task_config = get<JointTaskConfig>(task_config);
			auto joint_task =
				_robot_controllers.at(_active_controller_name)
					->getJointTaskByName(joint_task_config.task_name);

			// dynamic decoupling
			if (joint_task_config.use_dynamic_decoupling) {
				joint_task->setDynamicDecouplingType(
					SaiPrimitives::DynamicDecouplingType::
						BOUNDED_INERTIA_ESTIMATES);
				joint_task->setBoundedInertiaEstimateThreshold(
					joint_task_config.bie_threshold);
			} else {
				joint_task->setDynamicDecouplingType(
					SaiPrimitives::DynamicDecouplingType::IMPEDANCE);
			}

			// velocity saturation
			const auto& velocity_saturation_config =
				joint_task_config.velocity_saturation_config;
			if (velocity_saturation_config.enabled) {
				joint_task->enableVelocitySaturation(
					velocity_saturation_config.velocity_limit);
			} else {
				joint_task->disableVelocitySaturation();
			}

			// otg
			const auto& otg_config = joint_task_config.otg_config;
			if (!otg_config.enabled) {
				joint_task->disableInternalOtg();
			} else if (otg_config.enabled && !otg_config.jerk_limited) {
				joint_task->enableInternalOtgAccelerationLimited(
					otg_config.limits.max_velocity,
					otg_config.limits.max_acceleration);
			} else {
				joint_task->enableInternalOtgJerkLimited(
					otg_config.limits.max_velocity,
					otg_config.limits.max_acceleration,
					otg_config.limits.max_jerk);
			}

			// gains
			const auto& gains_config = joint_task_config.gains_config;
			if (gains_config.safety_checks_enabled) {
				joint_task->setGains(gains_config.kp, gains_config.kv,
									 gains_config.ki);
			} else {
				joint_task->setGainsUnsafe(gains_config.kp, gains_config.kv,
										   gains_config.ki);
			}

			// inputs
			auto& joint_task_input = std::get<JointTaskInput>(
				_controller_inputs.at(_active_controller_name)
					.at(joint_task_config.task_name));
			joint_task->setGoalPosition(joint_task_input.goal_position);
			joint_task->setGoalVelocity(joint_task_input.goal_velocity);
			joint_task->setGoalAcceleration(joint_task_input.goal_acceleration);

			// monitoring data
			auto& joint_task_monitoring_data =
				std::get<JointTaskMonitoringData>(
					_controller_task_monitoring_data.at(_active_controller_name)
						.at(joint_task_config.task_name));
			joint_task_monitoring_data.setFromTask(joint_task);
		} else if (holds_alternative<MotionForceTaskConfig>(task_config)) {
			auto& motion_force_task_config =
				get<MotionForceTaskConfig>(task_config);
			auto motion_force_task =
				_robot_controllers.at(_active_controller_name)
					->getMotionForceTaskByName(
						motion_force_task_config.task_name);
			auto& motion_force_task_input = std::get<MotionForceTaskInput>(
				_controller_inputs.at(_active_controller_name)
					.at(motion_force_task_config.task_name));

			if (motion_force_task_config.use_dynamic_decoupling) {
				motion_force_task->setDynamicDecouplingType(
					SaiPrimitives::DynamicDecouplingType::
						BOUNDED_INERTIA_ESTIMATES);
				motion_force_task->setBoundedInertiaEstimateThreshold(
					motion_force_task_config.bie_threshold);
			} else {
				motion_force_task->setDynamicDecouplingType(
					SaiPrimitives::DynamicDecouplingType::IMPEDANCE);
			}

			// force control parametrization
			motion_force_task->setForceSensorFrame(
				motion_force_task_config.link_name,
				motion_force_task_config.force_control_config
					.force_sensor_frame);
			motion_force_task->setClosedLoopForceControl(
				motion_force_task_config.force_control_config
					.closed_loop_force_control);
			motion_force_task->setClosedLoopMomentControl(
				motion_force_task_config.force_control_config
					.closed_loop_force_control);

			const auto& force_space_param_config =
				motion_force_task_config.force_control_config
					.force_space_param_config;
			if (motion_force_task->parametrizeForceMotionSpaces(
					force_space_param_config.force_space_dimension,
					force_space_param_config.axis)) {
				reset_inputs = true;
				motion_force_task_input.goal_position =
					motion_force_task->getCurrentPosition();
				motion_force_task_input.goal_linear_velocity.setZero();
				motion_force_task_input.goal_linear_acceleration.setZero();
			}

			const auto& moment_space_param_config =
				motion_force_task_config.force_control_config
					.moment_space_param_config;
			if (motion_force_task->parametrizeMomentRotMotionSpaces(
					moment_space_param_config.force_space_dimension,
					moment_space_param_config.axis)) {
				reset_inputs = true;
				motion_force_task_input.goal_position =
					motion_force_task->getCurrentPosition();
				motion_force_task_input.goal_linear_velocity.setZero();
				motion_force_task_input.goal_linear_acceleration.setZero();
			}

			// velocity saturation
			const auto& velocity_saturation_config =
				motion_force_task_config.velocity_saturation_config;
			if (velocity_saturation_config.enabled) {
				motion_force_task->enableVelocitySaturation(
					velocity_saturation_config.linear_velocity_limit,
					velocity_saturation_config.angular_velocity_limit);
			} else {
				motion_force_task->disableVelocitySaturation();
			}

			// otg
			const auto& otg_config = motion_force_task_config.otg_config;
			if (!otg_config.enabled) {
				motion_force_task->disableInternalOtg();
			} else if (otg_config.enabled && !otg_config.jerk_limited) {
				motion_force_task->enableInternalOtgAccelerationLimited(
					otg_config.max_linear_velocity,
					otg_config.max_linear_acceleration,
					otg_config.max_angular_velocity,
					otg_config.max_angular_acceleration);
			} else {
				motion_force_task->enableInternalOtgJerkLimited(
					otg_config.max_linear_velocity,
					otg_config.max_linear_acceleration,
					otg_config.max_linear_jerk, otg_config.max_angular_velocity,
					otg_config.max_angular_acceleration,
					otg_config.max_angular_jerk);
			}

			// gains
			const auto& pos_gains_config =
				motion_force_task_config.position_gains_config;
			if (pos_gains_config.safety_checks_enabled) {
				motion_force_task->setPosControlGains(pos_gains_config.kp,
													  pos_gains_config.kv,
													  pos_gains_config.ki);
			} else {
				motion_force_task->setPosControlGainsUnsafe(
					pos_gains_config.kp, pos_gains_config.kv,
					pos_gains_config.ki);
			}

			const auto& ori_gains_config =
				motion_force_task_config.orientation_gains_config;
			if (ori_gains_config.safety_checks_enabled) {
				motion_force_task->setOriControlGains(ori_gains_config.kp,
													  ori_gains_config.kv,
													  ori_gains_config.ki);
			} else {
				motion_force_task->setOriControlGainsUnsafe(
					ori_gains_config.kp, ori_gains_config.kv,
					ori_gains_config.ki);
			}

			if (motion_force_task_config.force_control_config
					.closed_loop_force_control) {
				const auto& force_gains_config =
					motion_force_task_config.force_control_config
						.force_gains_config;
				motion_force_task->setForceControlGains(
					force_gains_config.kp(0), force_gains_config.kv(0),
					force_gains_config.ki(0));
				const auto& moment_gains_config =
					motion_force_task_config.force_control_config
						.moment_gains_config;
				motion_force_task->setMomentControlGains(
					moment_gains_config.kp(0), moment_gains_config.kv(0),
					moment_gains_config.ki(0));
			}

			// inputs
			motion_force_task->setGoalPosition(
				motion_force_task_input.goal_position);
			motion_force_task->setGoalLinearVelocity(
				motion_force_task_input.goal_linear_velocity);
			motion_force_task->setGoalLinearAcceleration(
				motion_force_task_input.goal_linear_acceleration);
			motion_force_task->setGoalOrientation(
				motion_force_task_input.goal_orientation);
			motion_force_task->setGoalAngularVelocity(
				motion_force_task_input.goal_angular_velocity);
			motion_force_task->setGoalAngularAcceleration(
				motion_force_task_input.goal_angular_acceleration);
			motion_force_task->setGoalForce(
				motion_force_task_input.desired_force);
			motion_force_task->setGoalMoment(
				motion_force_task_input.desired_moment);
			motion_force_task->updateSensedForceAndMoment(
				motion_force_task_input.sensed_force_sensor_frame,
				motion_force_task_input.sensed_moment_sensor_frame);

			// monitoring data
			auto& motion_force_task_monitoring_data =
				std::get<MotionForceTaskMonitoringData>(
					_controller_task_monitoring_data.at(_active_controller_name)
						.at(motion_force_task_config.task_name));
			motion_force_task_monitoring_data.setFromTask(motion_force_task);

			// reset inputs if needed
			// dont overwrite _reset_redis_inputs to false if reset_inputs is
			// false because it might have been set to true by a controller
			// switching
			if (reset_inputs) {
				_reset_redis_inputs = reset_inputs;
			}
		}
	}

	// logging
	switch (_logging_state) {
		case LoggingState::OFF:
			if (_logging_on) {
				_logging_state = LoggingState::START;
			}
			break;
		case LoggingState::START:
			_robot_logger->start(_config.logger_config.frequency);
			for (auto& task_loggers : _task_loggers) {
				for (auto& pair : task_loggers.second) {
					pair.second->start(_config.logger_config.frequency);
				}
			}
			_logging_state = LoggingState::ON;
			break;

		case LoggingState::ON:
			if (!_logging_on) {
				_logging_state = LoggingState::STOP;
			}
			break;

		case LoggingState::STOP:
			_robot_logger->stop();
			for (auto& task_loggers : _task_loggers) {
				for (auto& pair : task_loggers.second) {
					pair.second->stop();
				}
			}
			_logging_state = LoggingState::OFF;
			break;

		default:
			break;
	}
}

}  // namespace SaiInterfaces