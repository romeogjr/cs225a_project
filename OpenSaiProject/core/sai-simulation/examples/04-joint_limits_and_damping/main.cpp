#include <iostream>
#include <string>
#include <thread>

#include "SaiGraphics.h"
#include "SaiSimulation.h"
#include "unistd.h"

using namespace std;

const string world_fname =
	string(EXAMPLES_FOLDER) + "/04-joint_limits_and_damping/world.urdf";

bool fSimulationRunning = false;

// sim
void simulation(shared_ptr<SaiSimulation::SaiSimulation> sim);

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["EXAMPLE_04_FOLDER"] =
		string(EXAMPLES_FOLDER) + "/04-joint_limits_and_damping";
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load simulation world
	auto sim = make_shared<SaiSimulation::SaiSimulation>(world_fname, false);
	const string robot_name = sim->getRobotNames()[0];

	// load graphics scene
	auto graphics = make_shared<SaiGraphics::SaiGraphics>(
		world_fname, "sai-simulation example 01-fixed_joint", false);
	graphics->addUIForceInteraction(robot_name);

	cout << endl
		 << "This example simulates a double pendulum robot with joint limits. "
			"The robot will stop in its fall because of the limits, It is "
			"possible to interact with the robot using right click to bring "
			"it up to its upper limit as well. After some time, the joint "
			"limits are disabled in the simulation. In addition. damping on "
			"the joints is enabled by setting the <dynamics damping=\" ... "
			"\"/> in the robot urdf file for the joints"
		 << endl
		 << endl;

	// start the simulation
	thread sim_thread(simulation, sim);

	Eigen::VectorXd torques = Eigen::VectorXd::Zero(sim->dof(robot_name));

	// while window is open:
	while (graphics->isWindowOpen()) {
		// apply joint torque
		sim->setJointTorques(robot_name, torques);

		// update graphics.
		graphics->updateRobotGraphics(robot_name,
									  sim->getJointPositions(robot_name));
		graphics->renderGraphicsWorld();
		torques = graphics->getUITorques(robot_name);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(shared_ptr<SaiSimulation::SaiSimulation> sim) {
	fSimulationRunning = true;
	double timestep = sim->timestep();

	unsigned long long sim_counter = 0;
	while (fSimulationRunning) {
		usleep(timestep * 1e6);
		// integrate forward
		sim->integrate();

		if (sim_counter == 3000) {	// disable joint limits after ~3 seconds
			std::cout << "\ndisabling joint limits\n" << std::endl;
			sim->disableJointLimits(sim->getRobotNames()[0]);
		}
		sim_counter++;
	}
}