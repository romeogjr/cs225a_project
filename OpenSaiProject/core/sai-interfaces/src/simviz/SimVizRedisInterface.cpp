#include "SimVizRedisInterface.h"

#include <signal.h>

#include <filesystem>
#include <glaze/glaze.hpp>

namespace SaiInterfaces {

namespace {
bool external_stop_signal = false;
void stop(int i) { external_stop_signal = true; }

const std::string sim_param_group_name = "simviz_redis_sim_param_group_name";

// redis keys
const std::string group_name = "simviz_redis_group_name";

const double viz_refresh_rate = 30.0;  // Hz

}  // namespace

SimVizRedisInterface::SimVizRedisInterface(const SimVizConfig& config,
										   const bool setup_signal_handler)
	: _config(config),
	  _new_config(config),
	  _pause(false),
	  _reset(false),
	  _can_reset(true),
	  _enable_grav_comp(true),
	  _logging_on(false),
	  _reset_complete(false),
	  _logging_state(LoggingState::OFF) {
	_graphics = std::make_unique<SaiGraphics::SaiGraphics>(
		_config.world_file, "sai world", false);
	_simulation = std::make_unique<SaiSimulation::SaiSimulation>(
		_config.world_file, false);

	_redis_client = std::make_unique<SaiCommon::RedisClient>(
		_config.redis_config.redis_namespace_prefix);
	_redis_client->connect(_config.redis_config.redis_ip,
						   _config.redis_config.redis_port);
	_redis_client->createNewSendGroup(group_name);
	_redis_client->createNewReceiveGroup(group_name);
	initializeRedisDatabase();
	resetInternal();

	if (setup_signal_handler) {
		signal(SIGABRT, &stop);
		signal(SIGTERM, &stop);
		signal(SIGINT, &stop);
	}
}

std::vector<std::string> SimVizRedisInterface::getRobotNames() const {
	return _simulation->getRobotNames();
}

std::vector<std::string> SimVizRedisInterface::getObjectNames() const {
	return _simulation->getObjectNames();
}

void SimVizRedisInterface::reset(const SimVizConfig& config) {
	_reset_complete = false;
	_new_config = config;
	_reset = true;
}

void SimVizRedisInterface::resetInternal() {
	_simulation->setTimestep(_config.timestep);
	_simulation->setCollisionRestitution(_config.global_collision_restitution);
	_simulation->setCoeffFrictionStatic(_config.global_friction_coefficient);
	_simulation->setCoeffFrictionDynamic(_config.global_friction_coefficient);

	_redis_client->setBool("simviz::gravity_comp_enabled",
						   _config.enable_gravity_compensation);
	_simulation->enableGravityCompensation(_config.enable_gravity_compensation);

	_robot_ui_torques.clear();
	_object_ui_torques.clear();
	_redis_client->deleteSendGroup(group_name);
	_redis_client->deleteReceiveGroup(group_name);
	_redis_client->createNewSendGroup(group_name);
	_redis_client->createNewReceiveGroup(group_name);

	_robot_control_torques.clear();
	_robot_q.clear();
	_robot_dq.clear();
	_robot_M.clear();
	_object_pose.clear();
	_object_vel.clear();
	_force_sensor_data.clear();

	_model_specific_params_string.clear();

	_loggers.clear();
	if (!std::filesystem::exists(_config.logger_config.folder_name)) {
		std::filesystem::create_directories(_config.logger_config.folder_name);
	}

	_logging_on = _config.logger_config.start_with_logger_on;
	_redis_client->setBool("simviz::logging_on", _logging_on);
	_logging_state = _logging_on ? LoggingState::START : LoggingState::OFF;

	for (auto& robot_name : _simulation->getRobotNames()) {
		// dof and joint positions and velocities initially
		const int robot_dof = _simulation->dof(robot_name);
		_robot_q[robot_name] = _simulation->getJointPositions(robot_name);
		_robot_dq[robot_name] = _simulation->getJointVelocities(robot_name);
		if (_config.publish_mass_matrices_to_redis) {
			_robot_M[robot_name] =
				_simulation->computeAndGetMassMatrix(robot_name);
		}

		// logger for all modes
		_loggers[robot_name] = std::make_unique<SaiCommon::Logger>(
			_config.logger_config.folder_name + "/" + robot_name + "_simviz",
			_config.logger_config.add_timestamp_to_filename);
		_loggers.at(robot_name)
			->addToLog(_robot_q.at(robot_name), "joint_positions");
		_loggers.at(robot_name)
			->addToLog(_robot_dq.at(robot_name), "joint_velocities");
		if (_config.publish_mass_matrices_to_redis) {
			_loggers.at(robot_name)
				->addToLog(_robot_M.at(robot_name), "mass_matrix");
		}

		// model specific parameters
		// if it does not exists yet, create it with the default and global
		// values
		if (_config.model_specific_dynamic_and_rendering_params.find(
				robot_name) ==
			_config.model_specific_dynamic_and_rendering_params.end()) {
			DynamicAndRenderingParams dynamic_and_rendering_params;
			dynamic_and_rendering_params.joint_limits_enabled =
				_config.enable_joint_limits;
			dynamic_and_rendering_params.collision_restitution_coefficient =
				_config.global_collision_restitution;
			dynamic_and_rendering_params.friction_coefficient =
				_config.global_friction_coefficient;
			_config.model_specific_dynamic_and_rendering_params[robot_name] =
				dynamic_and_rendering_params;
		}
		setModelSpecificParametersFromConfig(robot_name);

		// dynamic parameters receive via redis
		_model_specific_params_string[robot_name] = glz::write_json(
			_config.model_specific_dynamic_and_rendering_params.at(robot_name));
		_redis_client->addToReceiveGroup(
			"simviz::" + robot_name + "::model_specific_parameters",
			_model_specific_params_string.at(robot_name), group_name);

		// setup if there is simulation
		if (_config.mode != SimVizMode::VIZ_ONLY) {
			_graphics->addUIForceInteraction(robot_name);
			_robot_ui_torques[robot_name] = VectorXd::Zero(robot_dof);
			_robot_control_torques[robot_name] = VectorXd::Zero(robot_dof);

			// redis
			_redis_client->addToReceiveGroup(
				"commands::" + robot_name + "::control_torques",
				_robot_control_torques.at(robot_name), group_name);
			_redis_client->addToSendGroup(
				"sensors::" + robot_name + "::joint_positions",
				_robot_q.at(robot_name), group_name);
			_redis_client->addToSendGroup(
				"sensors::" + robot_name + "::joint_velocities",
				_robot_dq.at(robot_name), group_name);
			if (_config.publish_mass_matrices_to_redis) {
				_redis_client->addToSendGroup(
					"sensors::" + robot_name + "::model::mass_matrix",
					_robot_M.at(robot_name), group_name);
			}

			// logger
			_loggers.at(robot_name)
				->addToLog(_robot_control_torques.at(robot_name),
						   "control_torques");
			_loggers.at(robot_name)
				->addToLog(_robot_ui_torques.at(robot_name), "ui_torques");
		} else {  // setup for viz only
			_redis_client->addToReceiveGroup(
				"sensors::" + robot_name + "::joint_positions",
				_robot_q.at(robot_name), group_name);
			_redis_client->addToReceiveGroup(
				"sensors::" + robot_name + "::joint_velocities",
				_robot_dq.at(robot_name), group_name);
		}
	}

	for (auto& object_name : _simulation->getObjectNames()) {
		// initial object pose and velocity
		_object_pose[object_name] =
			_simulation->getObjectPose(object_name).matrix();
		_object_vel[object_name] = _simulation->getObjectVelocity(object_name);

		// logger for all modes
		_loggers[object_name] = std::make_unique<SaiCommon::Logger>(
			_config.logger_config.folder_name + "/" + object_name + "_simviz",
			_config.logger_config.add_timestamp_to_filename);
		_loggers.at(object_name)
			->addToLog(_object_pose.at(object_name), "pose");
		_loggers.at(object_name)
			->addToLog(_object_vel.at(object_name), "velocity");

		// model specific dynamic and rendering parameters
		if (_config.model_specific_dynamic_and_rendering_params.find(
				object_name) ==
			_config.model_specific_dynamic_and_rendering_params.end()) {
			DynamicAndRenderingParams dynamic_and_rendering_params;
			dynamic_and_rendering_params.collision_restitution_coefficient =
				_config.global_collision_restitution;
			dynamic_and_rendering_params.friction_coefficient =
				_config.global_friction_coefficient;
			_config.model_specific_dynamic_and_rendering_params[object_name] =
				dynamic_and_rendering_params;
		}
		setModelSpecificParametersFromConfig(object_name);

		// dynamic parameters receive via redis
		_model_specific_params_string[object_name] = glz::write_json(
			_config.model_specific_dynamic_and_rendering_params.at(
				object_name));
		_redis_client->addToReceiveGroup(
			"simviz::" + object_name + "::model_specific_parameters",
			_model_specific_params_string.at(object_name), group_name);

		// setup if there is simulation
		if (_config.mode != SimVizMode::VIZ_ONLY) {
			_redis_client->addToSendGroup(
				"sensors::" + object_name + "::object_pose",
				_object_pose.at(object_name), group_name);
			_redis_client->addToSendGroup(
				"sensors::" + object_name + "::object_velocity",
				_object_vel.at(object_name), group_name);

			_graphics->addUIForceInteraction(object_name);
			_object_ui_torques[object_name] = Eigen::VectorXd::Zero(6);

			_loggers.at(object_name)
				->addToLog(_object_ui_torques.at(object_name), "ui_torques");
		} else {  // setup for viz only
			_redis_client->addToReceiveGroup(
				"sensors::" + object_name + "::object_pose",
				_object_pose.at(object_name), group_name);
			_redis_client->addToReceiveGroup(
				"sensors::" + object_name + "::object_velocity",
				_object_vel.at(object_name), group_name);
		}
	}

	// force sensors only if there is simulation
	if (_config.mode != SimVizMode::VIZ_ONLY) {
		for (const auto& force_sensor_config : _config.force_sensors) {
			if (force_sensor_config.link_name == "") {
				_simulation->addSimulatedForceSensor(
					force_sensor_config.robot_or_object_name,
					force_sensor_config.transform_in_link,
					force_sensor_config.cutoff_frequency);
			} else {
				_simulation->addSimulatedForceSensor(
					force_sensor_config.robot_or_object_name,
					force_sensor_config.link_name,
					force_sensor_config.transform_in_link,
					force_sensor_config.cutoff_frequency);
			}
		}

		_force_sensor_data = _simulation->getAllForceSensorData();
		for (auto& force_sensor_data : _force_sensor_data) {
			_graphics->addForceSensorDisplay(force_sensor_data);
			if (force_sensor_data.link_name ==
				SaiSimulation::object_link_name) {
				_redis_client->addToSendGroup(
					"sensors::" + force_sensor_data.robot_or_object_name +
						"::ft_sensor::force",
					force_sensor_data.force_local_frame, group_name);
				_redis_client->addToSendGroup(
					"sensors::" + force_sensor_data.robot_or_object_name +
						"::ft_sensor::moment",
					force_sensor_data.moment_local_frame, group_name);

				_loggers.at(force_sensor_data.robot_or_object_name)
					->addToLog(force_sensor_data.force_local_frame,
							   "sensed_force");
				_loggers.at(force_sensor_data.robot_or_object_name)
					->addToLog(force_sensor_data.moment_local_frame,
							   "sensed_moment");
			} else {
				_redis_client->addToSendGroup(
					"sensors::" + force_sensor_data.robot_or_object_name +
						"::ft_sensor::" + force_sensor_data.link_name +
						"::force",
					force_sensor_data.force_local_frame, group_name);
				_redis_client->addToSendGroup(
					"sensors::" + force_sensor_data.robot_or_object_name +
						"::ft_sensor::" + force_sensor_data.link_name +
						"::moment",
					force_sensor_data.moment_local_frame, group_name);

				_loggers.at(force_sensor_data.robot_or_object_name)
					->addToLog(force_sensor_data.force_local_frame,
							   force_sensor_data.link_name + "_sensed_force");
				_loggers.at(force_sensor_data.robot_or_object_name)
					->addToLog(force_sensor_data.moment_local_frame,
							   force_sensor_data.link_name + "_sensed_moment");
			}
		}
	}

	_reset_complete = true;
}

void SimVizRedisInterface::initializeRedisDatabase() {
	_redis_client->createNewReceiveGroup(sim_param_group_name);
	_redis_client->addToReceiveGroup("simviz::pause", _pause,
									 sim_param_group_name);
	_redis_client->addToReceiveGroup("simviz::gravity_comp_enabled",
									 _enable_grav_comp, sim_param_group_name);
	_redis_client->addToReceiveGroup("simviz::logging_on", _logging_on,
									 sim_param_group_name);
}

void SimVizRedisInterface::run(const std::atomic<bool>& user_stop_signal) {
	std::thread sim_thread;
	std::thread redis_communication_thread;
	redis_communication_thread =
		std::thread(&SimVizRedisInterface::redisCommunicationLoopRun, this,
					std::ref(user_stop_signal));
	sim_thread = std::thread(&SimVizRedisInterface::simLoopRun, this,
							 std::ref(user_stop_signal));
	vizLoopRun(user_stop_signal);
	sim_thread.join();
	redis_communication_thread.join();
}

void SimVizRedisInterface::vizLoopRun(
	const std::atomic<bool>& user_stop_signal) {
	SaiCommon::LoopTimer timer(viz_refresh_rate);

	while (!user_stop_signal && !external_stop_signal) {
		timer.waitForNextLoop();
		// ensure a minimum amount of sleep time to let the communication thread
		// acquire the parametrization mutex when needed, even if the graphics
		// rendering takes up the whole time allowed for the loop
		this_thread::sleep_for(chrono::microseconds(50));

		if (_config.mode == SimVizMode::SIM_ONLY) {
			_graphics->renderBlackScreen();
			continue;
		}

		if (_graphics->isWindowOpen()) {
			std::lock_guard<std::mutex> lock(_mutex_parametrization);
			for (auto& robot_name : _graphics->getRobotNames()) {
				_graphics->updateRobotGraphics(robot_name,
											   _robot_q.at(robot_name),
											   _robot_dq.at(robot_name));
			}
			for (auto& object_name : _graphics->getObjectNames()) {
				_graphics->updateObjectGraphics(
					object_name, Eigen::Affine3d(_object_pose.at(object_name)),
					_object_vel.at(object_name));
			}
			if (_config.mode != SimVizMode::VIZ_ONLY) {
				for (auto& force_sensor_data : _force_sensor_data) {
					_graphics->updateDisplayedForceSensor(force_sensor_data);
				}
			}
			_graphics->renderGraphicsWorld();
			if (_config.mode != SimVizMode::VIZ_ONLY) {
				for (auto& robot_name : _graphics->getRobotNames()) {
					std::lock_guard<std::mutex> lock(_mutex_torques);
					_robot_ui_torques.at(robot_name) =
						_graphics->getUITorques(robot_name);
				}
				for (auto& object_name : _graphics->getObjectNames()) {
					std::lock_guard<std::mutex> lock(_mutex_torques);
					_object_ui_torques.at(object_name) =
						_graphics->getUITorques(object_name);
				}
			}
		} else {
			external_stop_signal = true;
		}
	}
	timer.stop();
	external_stop_signal = true;
}

double SimVizRedisInterface::computeCommunicationLoopFrequency() const {
	double communication_frequency;
	switch (_config.mode) {
		case SimVizMode::SIMVIZ:
			communication_frequency =
				max(_config.speedup_factor / _simulation->timestep(),
					viz_refresh_rate);
			break;
		case SimVizMode::SIM_ONLY:
			communication_frequency =
				_config.speedup_factor / _simulation->timestep();
			break;
		case SimVizMode::VIZ_ONLY:
			communication_frequency = viz_refresh_rate;
			break;
		default:
			throw std::runtime_error("Invalid mode for redis communication.");
			break;
	}
	return communication_frequency;
}

void SimVizRedisInterface::redisCommunicationLoopRun(
	const std::atomic<bool>& user_stop_signal) {
	_communication_timer =
		make_unique<SaiCommon::LoopTimer>(computeCommunicationLoopFrequency());
	_communication_timer->setTimerName(
		"Simviz Redis Interface Communication Loop Timer");

	while (!user_stop_signal && !external_stop_signal) {
		_communication_timer->waitForNextLoop();
		_redis_client->receiveAllFromGroup(sim_param_group_name);
		processSimParametrization();

		_redis_client->receiveAllFromGroup(group_name);

		if (_config.mode != SimVizMode::VIZ_ONLY) {
			_redis_client->sendAllFromGroup(group_name);
		}
	}
	_communication_timer->stop();
	external_stop_signal = true;

	// _communication_timer->printInfoPostRun();

	for (auto& logger : _loggers) {
		logger.second->stop();
	}
}

void SimVizRedisInterface::simLoopRun(
	const std::atomic<bool>& user_stop_signal) {
	_sim_timer = std::make_unique<SaiCommon::LoopTimer>(
		_config.speedup_factor / _simulation->timestep());
	_sim_timer->setTimerName("Simviz Redis Interface Sim Loop Timer");

	while (!user_stop_signal && !external_stop_signal) {
		_sim_timer->waitForNextLoop();

		if (_config.mode == SimVizMode::VIZ_ONLY) {
			continue;
		}

		if (_reset) {  // wait for the end of the reset
			continue;
		}

		if (_pause) {
			_sim_timer->stop();
			_simulation->pause();
		} else {
			_can_reset = false;
			if (_simulation->isPaused()) {
				_simulation->unpause();
				_sim_timer->reinitializeTimer();
			}
			for (auto& robot_name : _simulation->getRobotNames()) {
				std::lock_guard<std::mutex> lock(_mutex_torques);
				_simulation->setJointTorques(
					robot_name, _robot_ui_torques.at(robot_name) +
									_robot_control_torques.at(robot_name));
			}
			for (auto& object_name : _simulation->getObjectNames()) {
				std::lock_guard<std::mutex> lock(_mutex_torques);
				_simulation->setObjectForceTorque(
					object_name, _object_ui_torques.at(object_name));
			}

			_simulation->integrate();

			for (auto& robot_name : _simulation->getRobotNames()) {
				_robot_q.at(robot_name) =
					_simulation->getJointPositions(robot_name);
				_robot_dq.at(robot_name) =
					_simulation->getJointVelocities(robot_name);
				if (_config.publish_mass_matrices_to_redis) {
					_robot_M.at(robot_name) =
						_simulation->computeAndGetMassMatrix(robot_name);
				}
			}
			for (auto& object_name : _simulation->getObjectNames()) {
				_object_pose.at(object_name) =
					_simulation->getObjectPose(object_name).matrix();
				_object_vel.at(object_name) =
					_simulation->getObjectVelocity(object_name);
			}
			_force_sensor_data = _simulation->getAllForceSensorData();
			_can_reset = true;
		}
	}
	_sim_timer->stop();
	external_stop_signal = true;

	if (_config.mode != SimVizMode::VIZ_ONLY) {
		_sim_timer->printInfoPostRun();
	}
}

void SimVizRedisInterface::processSimParametrization() {
	if (_reset) {
		while (!_can_reset) {
			this_thread::sleep_for(chrono::microseconds(50));
		}
		std::lock_guard<mutex> lock(_mutex_parametrization);
		_config = _new_config;
		_simulation->resetWorld(_config.world_file);
		_graphics->resetWorld(_config.world_file);
		resetInternal();
		_sim_timer->resetLoopFrequency(_config.speedup_factor /
									   _simulation->timestep());
		_sim_timer->reinitializeTimer(5e6);
		_communication_timer->resetLoopFrequency(
			computeCommunicationLoopFrequency());
		_communication_timer->reinitializeTimer(5e6);
		_reset = false;
	}

	if (_enable_grav_comp != _simulation->isGravityCompensationEnabled()) {
		_simulation->enableGravityCompensation(_enable_grav_comp);
	}

	for (auto& pair : _model_specific_params_string) {
		try {
			auto result =
				glz::read_json<DynamicAndRenderingParams>(pair.second);
			if (!result) {
				throw std::runtime_error(
					"WARNING: Could not parse model "
					"specific parameters for model: " +
					pair.first + " from redis string: " + pair.second +
					". Keeping previous values");
			}
			if (_config.model_specific_dynamic_and_rendering_params.at(
					pair.first) != result.value()) {
				_config
					.model_specific_dynamic_and_rendering_params[pair.first] =
					result.value();
				setModelSpecificParametersFromConfig(pair.first);
			}
		} catch (const std::exception& e) {
			std::cerr << "Error: " << e.what() << std::endl;
			pair.second = glz::write_json(
				_config.model_specific_dynamic_and_rendering_params.at(
					pair.first));
			_redis_client->set("simviz::model_specific_params::" + pair.first,
							   pair.second);
		}
	}

	switch (_logging_state) {
		case LoggingState::OFF:
			if (_logging_on) {
				_logging_state = LoggingState::START;
			}
			break;
		case LoggingState::START:
			for (auto& logger : _loggers) {
				logger.second->start(_config.logger_config.frequency);
			}
			_logging_state = LoggingState::ON;
			break;

		case LoggingState::ON:
			if (!_logging_on) {
				_logging_state = LoggingState::STOP;
			}
			break;

		case LoggingState::STOP:
			for (auto& logger : _loggers) {
				logger.second->stop();
			}
			_logging_state = LoggingState::OFF;
			break;

		default:
			break;
	}
}

void SimVizRedisInterface::setModelSpecificParametersFromConfig(
	const std::string& model_name) {
	if (!_simulation->modelExistsInWorld(model_name)) {
		throw std::runtime_error("Model " + model_name +
								 " does not exist in the world, cannot set "
								 "model specific parameters.");
	}

	const auto model_specific_params =
		_config.model_specific_dynamic_and_rendering_params.at(model_name);
	_simulation->setDynamicsEnabled(model_specific_params.dynamics_enabled,
									model_name);
	_graphics->setRenderingEnabled(model_specific_params.rendering_enabled,
								   model_name);
	_simulation->setCollisionRestitution(
		model_specific_params.collision_restitution_coefficient, model_name);
	_simulation->setCoeffFrictionStatic(
		model_specific_params.friction_coefficient, model_name);
	_graphics->showWireMesh(model_specific_params.wire_mesh_rendering_mode,
							model_name);
	_graphics->showLinkFrame(model_specific_params.frames_rendering_enabled,
							 model_name, "",
							 model_specific_params.frames_size_when_rendering);

	if (_simulation->robotExistsInWorld(model_name)) {
		model_specific_params.joint_limits_enabled
			? _simulation->enableJointLimits(model_name)
			: _simulation->disableJointLimits(model_name);
	}
}

}  // namespace SaiInterfaces