#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>

#include "SaiGraphics.h"
#include "SaiPrimitives.h"
#include "SaiSimulation.h"
#include "redis/RedisClient.h"
#include "redis/keys/chai_haptic_devices_driver.h"
#include "timer/LoopTimer.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;
using namespace SaiCommon::ChaiHapticDriverKeys;

namespace {
const string world_file = "${EXAMPLE_17_FOLDER}/world.urdf";
const string robot_file = "${EXAMPLE_17_FOLDER}/panda_arm.urdf";
const string robot_name = "PANDA";
const string link_name = "end-effector";

// mutex for control torques
mutex mtx;

// map of flags for key presses
map<int, bool> key_pressed = {
	{GLFW_KEY_P, false},
	{GLFW_KEY_L, false},
	{GLFW_KEY_W, false},
	{GLFW_KEY_O, false},
};
map<int, bool> key_was_pressed = key_pressed;

}  // namespace

// Create simulation and control function
void runSim(shared_ptr<SaiSimulation::SaiSimulation> sim);
void runControl(shared_ptr<SaiSimulation::SaiSimulation> sim);

// robot joint data
VectorXd robot_control_torques = Eigen::VectorXd::Zero(7);

int main() {
	SaiModel::URDF_FOLDERS["EXAMPLE_17_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/17-bilateral_teleop_with_POPC";
	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load simulation world
	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);
	sim->addSimulatedForceSensor(robot_name, link_name, Affine3d::Identity(),
								 55.0);
	sim->setCoeffFrictionStatic(0.0);

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);

	// Run simulation and control threads
	thread sim_thread(runSim, sim);
	thread control_thread(runControl, sim);

	// graphics timer
	SaiCommon::LoopTimer graphicsTimer(30.0, 1e6);

	while (graphics->isWindowOpen()) {
		graphicsTimer.waitForNextLoop();

		for (auto& key : key_pressed) {
			key_pressed[key.first] = graphics->isKeyPressed(key.first);
		}

		graphics->updateRobotGraphics(robot_name,
									  sim->getJointPositions(robot_name));
		graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
		graphics->renderGraphicsWorld();
	}

	// stop simulation and control threads
	fSimulationRunning = false;
	sim_thread.join();
	control_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
////// Simulation thread //////
//------------------------------------------------------------------------------
void runSim(shared_ptr<SaiSimulation::SaiSimulation> sim) {
	// create a timer
	SaiCommon::LoopTimer simTimer(1.0 / sim->timestep(), 1e6);

	fSimulationRunning = true;

	while (fSimulationRunning) {
		simTimer.waitForNextLoop();
		{
			lock_guard<mutex> lock(mtx);
			sim->setJointTorques(robot_name, robot_control_torques);
		}
		sim->integrate();
	}

	cout << "simulation timer stats:" << endl;
	simTimer.printInfoPostRun();
}

//------------------------------------------------------------------------------
////// Control thread //////
//------------------------------------------------------------------------------
void runControl(shared_ptr<SaiSimulation::SaiSimulation> sim) {
	// redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// load robot
	const Affine3d T_world_robot = sim->getRobotBaseTransform(robot_name);
	auto robot = make_shared<SaiModel::SaiModel>(robot_file);
	robot->setTRobotBase(T_world_robot);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->setDq(sim->getJointVelocities(robot_name));
	robot->updateModel();

	// instructions
	cout
		<< "\nexmaple of a motion-motion controller to control a simulated "
		   "robot "
		   "with a haptic device, using direct force feedback with POPC "
		   "bilateral teleoperation to stabilize the contact force. The "
		   "controller will first bring the haptic device to its home pose and "
		   "then a press of the gripper or button is required to start "
		   "cotnrolling the robot."
		<< endl;
	cout << "Provided options:" << endl;
	cout << "1. press the device gripper/button to clutch the device (move the "
			"device without moving the robot) and release to get back the "
			"control of the robot."
		 << endl;
	cout << "2. Press 'p' to enable/disable plane guidance" << endl;
	cout << "3. Press 'l' to enable/disable line guidance" << endl;
	cout << "4. Press 'w' to enable/disable haptic workspace virtual limits"
		 << endl;
	cout << "5. Press 'o' to enable/disable orientation teleoperation" << endl;

	// create robot controller
	Affine3d compliant_frame = Affine3d::Identity();
	auto motion_force_task = make_shared<SaiPrimitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	motion_force_task->disableInternalOtg();
	motion_force_task->enableVelocitySaturation(0.9, M_PI);
	motion_force_task->setOriControlGains(200.0, 25.0);
	Vector3d prev_sensed_force = Vector3d::Zero();

	auto joint_task = make_shared<SaiPrimitives::JointTask>(robot);
	vector<shared_ptr<SaiPrimitives::TemplateTask>> task_list = {
		motion_force_task, joint_task};
	auto robot_controller =
		make_unique<SaiPrimitives::RobotController>(robot, task_list);

	// create haptic controller
	SaiPrimitives::HapticDeviceController::DeviceLimits device_limits(
		redis_client.getEigen(createRedisKey(MAX_STIFFNESS_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_DAMPING_KEY_SUFFIX, 0)),
		redis_client.getEigen(createRedisKey(MAX_FORCE_KEY_SUFFIX, 0)));
	auto haptic_controller =
		make_shared<SaiPrimitives::HapticDeviceController>(
			device_limits, robot->transformInWorld(link_name));
	haptic_controller->setScalingFactors(3.5);
	haptic_controller->setReductionFactorForce(0.7);
	VectorXd variable_damping_thresholds(2);
	variable_damping_thresholds << 0.25, 0.35;
	VectorXd variable_damping_gains(2);
	variable_damping_gains << 0, 20;
	haptic_controller->setVariableDampingGainsPos(variable_damping_thresholds,
												  variable_damping_gains);
	haptic_controller->setHapticControlType(
		SaiPrimitives::HapticControlType::HOMING);
	haptic_controller->disableOrientationTeleop();

	SaiPrimitives::HapticControllerInput haptic_input;
	SaiPrimitives::HapticControllerOutput haptic_output;
	bool haptic_button_was_pressed = false;
	int haptic_button_is_pressed = 0;
	redis_client.setInt(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
						haptic_button_is_pressed);
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 1);

	// create bilateral teleop POPC
	auto POPC_teleop = make_shared<SaiPrimitives::POPCBilateralTeleoperation>(
		motion_force_task, haptic_controller, 0.001);

	// setup redis communication
	redis_client.addToSendGroup(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
								haptic_output.device_command_force);
	redis_client.addToSendGroup(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
								haptic_output.device_command_moment);

	redis_client.addToReceiveGroup(createRedisKey(POSITION_KEY_SUFFIX, 0),
								   haptic_input.device_position);
	redis_client.addToReceiveGroup(createRedisKey(ROTATION_KEY_SUFFIX, 0),
								   haptic_input.device_orientation);
	redis_client.addToReceiveGroup(
		createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_linear_velocity);
	redis_client.addToReceiveGroup(
		createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX, 0),
		haptic_input.device_angular_velocity);
	redis_client.addToReceiveGroup(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX, 0),
								   haptic_button_is_pressed);

	// create a timer
	SaiCommon::LoopTimer controlTimer(1000.0, 1e6);

	while (fSimulationRunning) {
		// wait for next scheduled loop
		controlTimer.waitForNextLoop();

		// read robot data from simulation thread
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		robot_controller->updateControllerTaskModels();

		// read haptic device state from redis
		redis_client.receiveAllFromGroup();

		// compute haptic control
		haptic_input.robot_position = robot->positionInWorld(link_name);
		haptic_input.robot_orientation = robot->rotationInWorld(link_name);
		haptic_input.robot_linear_velocity =
			robot->linearVelocityInWorld(link_name);
		haptic_input.robot_angular_velocity =
			robot->angularVelocityInWorld(link_name);
		haptic_input.robot_sensed_force =
			motion_force_task->getSensedForceControlWorldFrame();
		haptic_input.robot_sensed_moment =
			motion_force_task->getSensedMomentControlWorldFrame();

		haptic_output = haptic_controller->computeHapticControl(haptic_input);

		// compute robot control
		motion_force_task->updateSensedForceAndMoment(
			sim->getSensedForce(robot_name, link_name),
			sim->getSensedMoment(robot_name, link_name));
		motion_force_task->setGoalPosition(haptic_output.robot_goal_position);
		motion_force_task->setGoalOrientation(
			haptic_output.robot_goal_orientation);

		// compute POPC
		auto POPC_force_moment =
			POPC_teleop->computeAdditionalHapticDampingForce();
		haptic_output.device_command_force += POPC_force_moment.first;

		redis_client.sendAllFromGroup();
		{
			lock_guard<mutex> lock(mtx);
			robot_control_torques = robot_controller->computeControlTorques();
		}

		// state machine for button presses
		if (haptic_controller->getHapticControlType() ==
				SaiPrimitives::HapticControlType::HOMING &&
			haptic_controller->getHomed() && haptic_button_is_pressed) {
			haptic_controller->setHapticControlType(
				SaiPrimitives::HapticControlType::MOTION_MOTION);
			haptic_controller->setDeviceControlGains(200.0, 15.0);
			cout << "haptic device homed" << endl;
		}

		if (haptic_controller->getHapticControlType() ==
				SaiPrimitives::HapticControlType::MOTION_MOTION &&
			haptic_button_is_pressed && !haptic_button_was_pressed) {
			haptic_controller->setHapticControlType(
				SaiPrimitives::HapticControlType::CLUTCH);
		} else if (haptic_controller->getHapticControlType() ==
					   SaiPrimitives::HapticControlType::CLUTCH &&
				   !haptic_button_is_pressed && haptic_button_was_pressed) {
			haptic_controller->setHapticControlType(
				SaiPrimitives::HapticControlType::MOTION_MOTION);
		} else if (key_pressed.at(GLFW_KEY_P) &&
				   !key_was_pressed.at(GLFW_KEY_P)) {
			if (haptic_controller->getPlaneGuidanceEnabled()) {
				cout << "disabling plane guidance" << endl;
				haptic_controller->disablePlaneGuidance();
			} else {
				cout << "enabling plane guidance" << endl;
				haptic_controller->enablePlaneGuidance(
					haptic_input.device_position, Vector3d::UnitZ());
			}
		} else if (key_pressed.at(GLFW_KEY_L) &&
				   !key_was_pressed.at(GLFW_KEY_L)) {
			if (haptic_controller->getLineGuidanceEnabled()) {
				cout << "disabling line guidance" << endl;
				haptic_controller->disableLineGuidance();
			} else {
				cout << "enabling line guidance" << endl;
				haptic_controller->enableLineGuidance(
					haptic_input.device_position, Vector3d::UnitZ());
			}
		} else if (key_pressed.at(GLFW_KEY_W) &&
				   !key_was_pressed.at(GLFW_KEY_W)) {
			if (haptic_controller->getHapticWorkspaceVirtualLimitsEnabled()) {
				cout << "disabling haptic workspace virtual limits" << endl;
				haptic_controller->disableHapticWorkspaceVirtualLimits();
			} else {
				cout << "enabling haptic workspace virtual limits" << endl;
				haptic_controller->enableHapticWorkspaceVirtualLimits(
					haptic_input.device_position.norm(), M_PI / 3);
			}
		} else if (key_pressed.at(GLFW_KEY_O) &&
				   !key_was_pressed.at(GLFW_KEY_O)) {
			if (haptic_controller->getOrientationTeleopEnabled()) {
				cout << "disabling orientation teleoperation" << endl;
				haptic_controller->disableOrientationTeleop();
			} else {
				cout << "enabling orientation teleoperation" << endl;
				haptic_controller->enableOrientationTeleop();
			}
		}

		haptic_button_was_pressed = haptic_button_is_pressed;
		key_was_pressed = key_pressed;
	}

	redis_client.setEigen(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setEigen(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX, 0),
						  Vector3d::Zero());
	redis_client.setInt(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX, 0), 0);

	cout << "control timer stats:" << endl;
	controlTimer.printInfoPostRun();
}