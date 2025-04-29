/*
 * Example of a controller for a Panda arm (7DoF robot) using a 6DoF
 * position plus orientation task and a joint task in its nullspace
 * to control the redundancy. The joint task activates after a few seconds.
 * Interpolation is not used. Instead, the trajectory is directly sent
 * to the robot.
 */

#include <math.h>
#include <signal.h>

#include <iostream>
#include <mutex>
#include <string>
#include <thread>

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

const string world_file = "${EXAMPLE_04_FOLDER}/world.urdf";
const string robot_file =
	"${SAI_MODEL_URDF_FOLDER}/panda/panda_arm_sphere.urdf";
const string robot_name = "PANDA";

// ui torques and control torques
VectorXd ui_torques;
VectorXd control_torques;

// mutex for global variables between different threads
mutex mutex_torques;

// simulation and control loop
void control(shared_ptr<SaiModel::SaiModel> robot,
			 shared_ptr<SaiSimulation::SaiSimulation> sim);
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim);

//------------ main function
int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_04_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/04-task_and_redundancy";
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

	// load robots
	auto robot = make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(sim->getJointPositions(robot_name));
	robot->updateModel();

	// intitialize global torques variables
	ui_torques = VectorXd::Zero(robot->dof());
	control_torques = VectorXd::Zero(robot->dof());

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);

	// while window is open:
	while (graphics->isWindowOpen()) {
		graphics->updateRobotGraphics(robot_name, robot->q());
		graphics->renderGraphicsWorld();
		{
			lock_guard<mutex> lock(mutex_torques);
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
	// update robot model and initialize control vectors
	robot->updateModel();
	int dof = robot->dof();
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// Position plus orientation task
	string link_name = "end-effector";
	Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.07);
	Affine3d compliant_frame = Affine3d(Translation3d(pos_in_link));
	auto motion_force_task = make_unique<SaiPrimitives::MotionForceTask>(
		robot, link_name, compliant_frame);
	VectorXd motion_force_task_torques = VectorXd::Zero(dof);
	motion_force_task->disableInternalOtg();

	// no gains setting here, using the default task values
	const Matrix3d initial_orientation = robot->rotation(link_name);
	const Vector3d initial_position = robot->position(link_name, pos_in_link);

	// joint task to control the redundancy
	// using default gains and interpolation settings
	auto joint_task = make_unique<SaiPrimitives::JointTask>(robot);
	VectorXd joint_task_torques = VectorXd::Zero(dof);

	VectorXd initial_q = robot->q();

	// create a loop timer
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// read joint positions, velocities, update model
		robot->setQ(sim->getJointPositions(robot_name));
		robot->setDq(sim->getJointVelocities(robot_name));
		robot->updateModel();

		// update tasks model. Order is important to define the hierarchy
		N_prec = MatrixXd::Identity(dof, dof);

		motion_force_task->updateTaskModel(N_prec);
		N_prec = motion_force_task->getTaskAndPreviousNullspace();
		// after each task, need to update the nullspace
		// of the previous tasks in order to garantee
		// the dyamic consistency

		joint_task->updateTaskModel(N_prec);

		// -------- set task goals and compute control torques
		// first the posori task.
		// orientation: oscillation around Y
		double w_ori_traj = 2 * M_PI * 0.2;
		double amp_ori_traj = M_PI / 8;
		double angle_ori_traj = amp_ori_traj * sin(w_ori_traj * time);
		double ang_vel_traj =
			amp_ori_traj * w_ori_traj * cos(w_ori_traj * time);
		double ang_accel_traj =
			amp_ori_traj * w_ori_traj * w_ori_traj * -sin(w_ori_traj * time);

		Matrix3d R =
			AngleAxisd(angle_ori_traj, Vector3d::UnitY()).toRotationMatrix();

		motion_force_task->setGoalOrientation(R.transpose() *
											  initial_orientation);
		motion_force_task->setGoalAngularVelocity(ang_vel_traj *
												  Vector3d::UnitY());
		motion_force_task->setGoalAngularAcceleration(ang_accel_traj *
													  Vector3d::UnitY());

		// position: circle in the y-z plane
		double radius_circle_pos = 0.05;
		double w_circle_pos = 2 * M_PI * 0.33;
		motion_force_task->setGoalPosition(
			initial_position +
			radius_circle_pos * Vector3d(0.0, sin(w_circle_pos * time),
										 1 - cos(w_circle_pos * time)));
		motion_force_task->setGoalLinearVelocity(
			radius_circle_pos * w_circle_pos *
			Vector3d(0.0, cos(w_circle_pos * time), sin(w_circle_pos * time)));
		motion_force_task->setGoalLinearAcceleration(
			radius_circle_pos * w_circle_pos * w_circle_pos *
			Vector3d(0.0, -sin(w_circle_pos * time), cos(w_circle_pos * time)));

		// compute torques for the different tasks
		motion_force_task_torques = motion_force_task->computeTorques();
		joint_task_torques = joint_task->computeTorques();

		// activate joint task only after 5 seconds and try to rotate the first
		// joint
		if (timer.elapsedCycles() < 5000) {
			joint_task_torques.setZero();
		}
		if (timer.elapsedCycles() == 5000) {
			joint_task->reInitializeTask();
			VectorXd goal_joint_pos = initial_q;
			goal_joint_pos(0) += 1.5;
			joint_task->setGoalPosition(goal_joint_pos);
		}

		//------ compute the final torques
		{
			lock_guard<mutex> lock(mutex_torques);
			control_torques = motion_force_task_torques + joint_task_torques;
		}

		// -------------------------------------------
		if (timer.elapsedCycles() % 500 == 0) {
			cout << "time: " << time << endl;
			cout << "position error : "
				 << (motion_force_task->getGoalPosition() -
					 motion_force_task->getCurrentPosition())
						.norm()
				 << endl;
			cout << endl;
		}
	}
	timer.stop();
	cout << "\nControl loop timer stats:\n";
	timer.printInfoPostRun();
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<SaiModel::SaiModel> robot,
				shared_ptr<SaiSimulation::SaiSimulation> sim) {
	fSimulationRunning = true;

	// create a timer
	double sim_freq = 2000;
	SaiCommon::LoopTimer timer(sim_freq);

	sim->setTimestep(1.0 / sim_freq);

	while (fSimulationRunning) {
		timer.waitForNextLoop();
		{
			lock_guard<mutex> lock(mutex_torques);
			sim->setJointTorques(robot_name, control_torques + ui_torques);
		}
		sim->integrate();
	}
	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
}