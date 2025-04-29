/*
 * Example of a controller for a Panda arm (7DoF robot)
 * performing a surface-surface alignment (zero moment control)
 * after reaching contact
 */

// Initialization is the same as previous examples
#include <math.h>
#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "RobotController.h"
#include "SaiGraphics.h"
#include "SaiModel.h"
#include "SaiSimulation.h"
#include "tasks/JointTask.h"
#include "tasks/MotionForceTask.h"
#include "timer/LoopTimer.h"
bool fSimulationRunning = false;
void sighandler(int) { fSimulationRunning = false; }

using namespace std;
using namespace Eigen;

const string world_file = "${EXAMPLE_07_FOLDER}/world.urdf";
const string robot_file = "${SAI_MODEL_URDF_FOLDER}/panda//panda_arm_box.urdf";
const string robot_name = "PANDA";
// need a second robot model for the plate
const string plate_file = "${EXAMPLE_07_FOLDER}/plate.urdf";
const string plate_name = "Plate";

// global variables for sensed force and moment
Vector3d sensed_force;
Vector3d sensed_moment;

VectorXd control_torques, ui_torques;
mutex torques_mutex;

// global variables for controller parametrization
const string link_name = "end-effector";
const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.04);
const Vector3d sensor_pos_in_link = Vector3d(0.0, 0.0, 0.0);

// state machine for control
#define GO_TO_CONTACT 0
#define CONTACT_CONTROL 1

// simulation and control loop
void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiModel::SaiModel> plate,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

//------------ main function
int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_07_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/07-surface_surface_contact";
	cout << "Loading URDF world model file: "
		 << SaiModel::ReplaceUrdfPathPrefix(world_file) << endl;

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(world_file);
	graphics->addUIForceInteraction(robot_name);

	// load simulation world
	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_file);
	sim->setCoeffFrictionStatic(0.3);
	sim->setCollisionRestitution(0);

	// load robot and plate
	auto robot = make_shared<SaiModel::SaiModel>(robot_file);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

	// load plate
	auto plate = make_shared<SaiModel::SaiModel>(plate_file);

	// create simulated force sensor
	Affine3d T_sensor = Affine3d::Identity();
	T_sensor.translation() = sensor_pos_in_link;
	sim->addSimulatedForceSensor(robot_name, link_name, T_sensor, 15.0);
	graphics->addForceSensorDisplay(sim->getAllForceSensorData()[0]);

	control_torques.setZero(robot->dof());
	ui_torques.setZero(robot->dof());

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, plate, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);

	// while window is open:
	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->updateRobotGraphics(plate_name, plate->q());
		graphics->updateDisplayedForceSensor(sim->getAllForceSensorData()[0]);
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(torques_mutex);
			ui_torques = graphics->getUITorques(robot_name);
		}
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim) {
	// prepare state machine
	int state = GO_TO_CONTACT;

	// update robot model and initialize control vectors
	robot->updateModel();
	int dof = robot->dof();

	// Position plus orientation task
	auto motion_force_task = make_shared<SaiPrimitives::MotionForceTask>(
		robot, link_name, Affine3d(Translation3d(pos_in_link)),
		"surface_alignment_task", true);
	motion_force_task->enablePassivity();
	motion_force_task->disableInternalOtg();
	VectorXd motion_force_task_torques = VectorXd::Zero(dof);
	// set the force sensor location for the contact part of the task
	motion_force_task->setForceSensorFrame(link_name, Affine3d::Identity());

	// no gains setting here, using the default task values
	const Matrix3d initial_orientation = robot->rotation(link_name);
	const Vector3d initial_position = robot->position(link_name, pos_in_link);
	Vector3d goal_position = initial_position;
	VectorXd initial_q = robot->q();

	// robot controller
	vector<shared_ptr<SaiPrimitives::TemplateTask>> task_list = {
		motion_force_task};
	auto robot_controller =
		make_unique<SaiPrimitives::RobotController>(robot, task_list);

	// create a loop timer
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	while (fSimulationRunning) {  // automatically set to false when simulation
								  // is quit
		timer.waitForNextLoop();

		// read joint positions, velocities, update model
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		// update force sensor values (needs to be the force applied by the
		// robot to the environment, in sensor frame)
		motion_force_task->updateSensedForceAndMoment(sensed_force,
													  sensed_moment);

		// update tasks model
		robot_controller->updateControllerTaskModels();

		// -------- set task goals in the state machine and compute control
		// torques
		if (state == GO_TO_CONTACT) {
			goal_position(2) -=
				0.00003;  // go down at 30 cm/s until contact is detected
			motion_force_task->setGoalPosition(goal_position);

			if (motion_force_task->getSensedForceControlWorldFrame()(2) <=
				-1.0) {
				// switch the local z axis to be force controlled and the local
				// x and y axis to be moment controlled
				motion_force_task->parametrizeForceMotionSpaces(
					1, Vector3d::UnitZ());
				motion_force_task->parametrizeMomentRotMotionSpaces(
					2, Vector3d::UnitZ());

				motion_force_task->setClosedLoopForceControl();
				motion_force_task->setClosedLoopMomentControl();

				// set the force and moment control set points and gains
				motion_force_task->setGoalForce(10.0 * Vector3d::UnitZ());
				motion_force_task->setGoalMoment(Vector3d::Zero());

				motion_force_task->setForceControlGains(0.7, 5.0, 1.5);
				motion_force_task->setMomentControlGains(0.7, 4.0, 1.5);

				// change the state of the state machine
				state = CONTACT_CONTROL;
			}
		} else if (state == CONTACT_CONTROL) {
			if (timer.elapsedCycles() % 1000 == 0) {
				cout << "Current force: "
					 << motion_force_task->getSensedForceControlWorldFrame()
							.transpose()
					 << endl;
				cout << "Current moment: "
					 << motion_force_task->getSensedMomentControlWorldFrame()
							.transpose()
					 << endl;
				cout << "Goal force: "
					 << motion_force_task->getGoalForce().transpose() << endl;
				cout << "Goal moment: "
					 << motion_force_task->getGoalMoment().transpose() << endl;
				cout << endl;
			}
		}

		//------ compute the final torques
		{
			lock_guard<mutex> lock(torques_mutex);
			control_torques = robot_controller->computeControlTorques();
		}
	}
	timer.stop();
	cout << "\nControl loop timer stats:\n";
	timer.printInfoPostRun();
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiModel::SaiModel> plate,
				shared_ptr<SaiSimulation::SaiSimulation> sim) {
	fSimulationRunning = true;

	// plate controller
	Vector2d plate_qd = Vector2d::Zero();
	Vector2d plate_torques = Vector2d::Zero();

	// create a timer
	double sim_freq = 2000;
	SaiCommon::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		double time = timer.elapsedSimTime();

		// force sensor update
		sensed_force = sim->getSensedForce(robot_name, link_name);
		sensed_moment = sim->getSensedMoment(robot_name, link_name);

		// plate controller
		plate->setQ(sim->getJointPositions(plate_name));
		plate->setDq(sim->getJointVelocities(plate_name));
		plate->updateKinematics();

		plate_qd(0) = 5.0 / 180.0 * M_PI * sin(2 * M_PI * 0.12 * time);
		plate_qd(1) = 7.0 / 180.0 * M_PI * sin(2 * M_PI * 0.08 * time);

		plate_torques = -1000.0 * (plate->q() - plate_qd) - 75.0 * plate->dq();

		sim->setJointTorques(plate_name, plate_torques);

		{
			lock_guard<mutex> lock(torques_mutex);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}

		// integrate forward
		sim->integrate();
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}