/**
 * @file controller.cpp
 * @brief Controller file
 * 
 */

#include <SaiModel.h>
#include "SaiPrimitives.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <Eigen/Geometry>

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;
using namespace SaiPrimitives;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

#include "redis_keys.h"

// States 
enum State {
	POSTURE = 0, 
	INITIAL_ROTATION,	
	INITIAL_APPROACH, 
	RETRACT
};

int main() {
	// Location of URDF files specifying world and robot information
	static const string robot_file = string(CS225A_URDF_FOLDER) + "/panda/panda_arm_hand.urdf";

	// initial state 
	int state = POSTURE;
	string controller_status = "1";
	
	// start redis client
	auto redis_client = SaiCommon::RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state and update the model
	auto robot = std::make_shared<SaiModel::SaiModel>(robot_file, false);
	robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
	robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);  // panda torques 
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// Postion of EE intilization 
	const string control_link = "link7";
	// Will change later on
	const Vector3d control_point = Vector3d(0.0, 0.0, 0.07); // change based on sponge?
	Affine3d compliant_frame = Affine3d::Identity();
	compliant_frame.translation() = control_point;
	auto pose_task = std::make_shared<SaiPrimitives::MotionForceTask>(robot, control_link, compliant_frame);
	// Will tune depending on the task
	pose_task->setPosControlGains(400, 40, 0);
	pose_task->setOriControlGains(400, 40, 0);

	Vector3d ee_pos;
	Matrix3d ee_ori;

	// joint task
	auto joint_task = std::make_shared<SaiPrimitives::JointTask>(robot);
	joint_task->setGains(400, 40, 0);

	VectorXd q_desired(dof);
	q_desired << -30.0, -15.0, -15.0, -105.0, 0.0, 90.0, 45.0;
	q_desired *= M_PI / 180.0;

	joint_task->setGoalPosition(q_desired);

	// create a loop timer
	runloop = true;
	double control_freq = 1000;
	SaiCommon::LoopTimer timer(control_freq, 1e6);

	while (runloop) {
		timer.waitForNextLoop();
		const double time = timer.elapsedSimTime();

		// update robot 
		robot->setQ(redis_client.getEigen(JOINT_ANGLES_KEY));
		robot->setDq(redis_client.getEigen(JOINT_VELOCITIES_KEY));
		robot->updateModel();
	
		if (state == POSTURE) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			command_torques = joint_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Posture To Motion" << endl;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();

				// ee_pos = robot->position(control_link, control_point);
				
				

				// pose_task->setGoalPosition(ee_pos - Vector3d(-0.1, -0.1, 0.1));
				// pose_task->setGoalOrientation(AngleAxisd(M_PI / 6, Vector3d::UnitX()).toRotationMatrix() * ee_ori);

				state = INITIAL_ROTATION;
			}

		} else if (state == INITIAL_ROTATION) {
			// update goal position and orientation

		


			// align sponge cylinder axis (local Z) to world Y (faces XZ plane)
			Eigen::Matrix3d sponge_ori = Eigen::AngleAxisd(-M_PI/2, Vector3d::UnitX()).toRotationMatrix();
			pose_task->setGoalOrientation(sponge_ori);
			
			
	

			// align sponge cylinder axis (local Z) to world Y (faces XZ plane)
			

			N_prec.setIdentity();
			// set pose task as priority 1
			pose_task->updateTaskModel(N_prec);
			// secondary priority: joint task in nullspace of pose
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			// set priority 1 as velocity saturation
			// set priority 2 as posture

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			// check if orientation goal reached
			ee_ori = robot->rotation(control_link);
			if ((ee_ori - sponge_ori).norm() < 1e-2) {
				cout << "Orientation Achieved" << endl;
				state = INITIAL_APPROACH;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();
			}
		} else if (state == INITIAL_APPROACH) {
			// update goal position and orientation

			Vector3d ee_pos_desired;
			ee_pos_desired << 0.5, 0.175, 0.6;

			Vector3d ee_ori_desired;
			ee_ori_desired << 0.0, 0.0, 1.5708;
			
			
			pose_task->setGoalPosition(ee_pos_desired);

			// align sponge cylinder axis (local Z) to world Y (faces XZ plane)
			

			N_prec.setIdentity();
			// set pose task as priority 1
			pose_task->updateTaskModel(N_prec);
			// secondary priority: joint task in nullspace of pose
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			// set priority 1 as velocity saturation
			// set priority 2 as posture

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();



		} else if (state == RETRACT) {
			// update task model 
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			command_torques = joint_task->computeTorques();

			if ((robot->q() - q_desired).norm() < 1e-2) {
				cout << "Motion To Posture" << endl;
				state = POSTURE;
			}
		}

		// execute redis write callback
		redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	}

	timer.stop();
	cout << "\nSimulation loop timer stats:\n";
	timer.printInfoPostRun();
	redis_client.setEigen(JOINT_TORQUES_COMMANDED_KEY, 0 * command_torques);  // back to floating

	return 0;
}
