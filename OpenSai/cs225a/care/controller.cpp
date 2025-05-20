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
	 INITIAL_APPROACH1, 
	 INITIAL_APPROACH2,
	 CLEAN_1, 
	 RETRACT, 
	 STOP
 };

 int prev_state = -1;
 
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
	 const string control_link   = "sponge";
	 const Vector3d control_point = Vector3d(0.0, 0.0, 0.1 + 0.03/2);
	 //     visual origin (0.1) + half the cylinder length (0.015) = 0.115
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
 
	 // Timer for clean 1
	 double clean1_start_time = -1.0;
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
 
				 state = INITIAL_ROTATION;
			 }
 
		 } else if (state == INITIAL_ROTATION) {
			 // update goal position and orientation
 
			 // align sponge cylinder axis (local Z) to world Y (faces XZ plane)
			 Eigen::Matrix3d sponge_ori = Eigen::AngleAxisd(-M_PI/2, Vector3d::UnitX()).toRotationMatrix();
			 pose_task->setGoalOrientation(sponge_ori);
			 
 
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
				 state = INITIAL_APPROACH1;
				 pose_task->reInitializeTask();
				 joint_task->reInitializeTask();
			 }
		 } else if (state == INITIAL_APPROACH1) {
			 // update goal position and orientation
 
			 // 1) set your desired goal
			 Vector3d ee_pos_desired = redis_client.getEigen(EE_POS_DESIRED_KEY);
			 pose_task->setGoalPosition(ee_pos_desired);
 
			 // 2) turn on velocity saturation (linear , angular )
			 pose_task->enableVelocitySaturation(0.1, 0.5);
 
			 // 3) build your task hierarchy as usual
			 N_prec.setIdentity();
			 pose_task->updateTaskModel(N_prec);
			 joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
 
			 // 4) compute torques (now with sat’d vels)
			 command_torques = pose_task->computeTorques() + joint_task->computeTorques();
 
			 // Print position				
			 Vector3d ee_pos_current = robot->position(control_link, control_point);
			 cout << "INITIAL_APPROACH1 | Current:  "
				 << ee_pos_current.transpose()
				 << "  Desired:  " << ee_pos_desired.transpose() 
				 << "  Err: " << (ee_pos_current - ee_pos_desired).norm()
				 << endl;
 
			 const double thresh = 1e-2;  // 1 cm on XY maybe?
			 if ((ee_pos_current - ee_pos_desired).norm() < thresh) {
				pose_task -> disableVelocitySaturation();
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();
				prev_state = state;
				state = INITIAL_APPROACH2;
				
			 }

			} else if (state == INITIAL_APPROACH2) {
				pose_task->setPosControlGains(0.0, 40.0, 0.0);
				pose_task->setGoalLinearVelocity(Eigen::Vector3d(0.0, 0.01, 0.0));

				N_prec.setIdentity();
				pose_task->updateTaskModel(N_prec);
				joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

				command_torques = pose_task->computeTorques() + joint_task->computeTorques();

				// Print position				
				Vector3d ee_curr_vel= robot->linearVelocity(control_link, control_point);
				Vector3d ee_pos_current = robot->position(control_link, control_point);
				cout << "INITIAL_APPROACH2 | Current:  "
				 << ee_pos_current.transpose()  
				 << ee_curr_vel.transpose() << endl;

				const double thresh = 1e-3;  // 1 cm on XY maybe?
				if (abs(ee_curr_vel.y()) < thresh) {
					cout << "Aligned at center height—starting CLEAN_1\n";
					prev_state = state;
					state = CLEAN_1;
					// reset tasks here
					clean1_start_time = -1.0;
					pose_task->reInitializeTask();
					joint_task->reInitializeTask();
				}
 
		 } else if (state == CLEAN_1) {
			 // initialize start time
			 if (clean1_start_time < 0.0) {
				 clean1_start_time = time;
			 }
			 double t_elapsed = time - clean1_start_time;
 
			 // get these points from redis keys
			 const double torso_center_z    = 0.6;     // from <origin xyz=…>
			 const double torso_half_height = 0.5 / 2; // box size z = 0.5
 
			 // ■ compute amplitude so sponge never crosses top/bottom
			 double amplitude_z = torso_half_height/2;
 
			 // ■ pick a frequency (e.g. 0.2 Hz ⇒ one full up+down every 5 s)
			 const double freq1  = .20;
			 double omega = 2.0 * M_PI * freq1;
 
			 // ■ desired Z = center + amplitude * sin(ω t)
			 double z_des = torso_center_z + amplitude_z * sin(omega * t_elapsed);
			//  double zdot_des = amplitude_z * cos(omega * t_elapsed) * omega;
			//  double zdotdot_des = -amplitude_z * sin(omega * t_elapsed) * omega * omega;

	 		double y_des = 0.173; // 0.173 is the position slightly into the human, depth of sponge + y (cm)
 
			 // ■ build the full 3D goal (keep your contact Y fixed)
			 Vector3d ee_cur_pos = robot->position(control_link, control_point);
			 Vector3d ee_cur_vel = robot->linearVelocity(control_link, control_point);
			 Vector3d goal;
			 goal << ee_cur_pos.x(), ee_cur_pos.y(), z_des;

			VectorXd kp_gain_orientation(3);
			 kp_gain_orientation << 10.0, 400.0, 10.0;
			 VectorXd kv_gain_orientation = VectorXd::Constant(3, 10.0);
			 VectorXd ki_gain_orientation = VectorXd::Zero(3);
			 pose_task->setOriControlGains(kp_gain_orientation, kv_gain_orientation, ki_gain_orientation);

			 VectorXd kp_gain(3);
			 kp_gain << 400.0, 10.0, 400.0;
			 VectorXd kv_gain = VectorXd::Constant(3, 10.0);
			 VectorXd ki_gain = VectorXd::Zero(3);
			 pose_task->setPosControlGains(kp_gain, kv_gain, ki_gain);
			 pose_task->setGoalPosition(goal);
			 //pose_task->setGoalLinearVelocity(Eigen::Vector3d(ee_cur_vel.x(), ee_cur_vel.y(), zdot_des));
			 //pose_task->setGoalLinearAcceleration(Vector3d(0.0, 0.0, zdotdot_des));
 
			 // ■ task hierarchy + torque
			 N_prec.setIdentity();
			 pose_task->updateTaskModel(N_prec);
			 joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			 command_torques = pose_task->computeTorques()
							 + joint_task->computeTorques();
 
			 // ■ debug print
			 cout << "CLEAN_1 | z_cur=" << ee_cur_pos.z()
				 << " z_des=" << z_des
				 << " err=" << fabs(ee_cur_pos.z()-z_des) << endl;
 
			 // ■ after 15 s, go to RETRACT
			 if (t_elapsed > 10.0) {
				 cout << "CLEAN_1 complete, switching to RETRACT\n";
				 state = RETRACT;
				 pose_task->reInitializeTask();
				 joint_task->reInitializeTask();
			 }
 
		 } else if (state == RETRACT) {
			 // 1) set your desired goal
			 Vector3d ee_cur_pos = robot->position(control_link, control_point);
			 Vector3d ee_pos_desired;
			 ee_pos_desired << ee_cur_pos.x(), 0.125, ee_cur_pos.z();
			 pose_task->setGoalPosition(ee_pos_desired);
 
			 // 2) turn on velocity saturation (linear , angular )
			 pose_task->enableVelocitySaturation(0.1, 0.5);
 
			 // 3) build your task hierarchy as usual
			 N_prec.setIdentity();
			 pose_task->updateTaskModel(N_prec);
			 joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
 
			 // 4) compute torques (now with sat’d vels)
			 command_torques = pose_task->computeTorques() + joint_task->computeTorques();
 
			 // Print position				
			 Vector3d ee_pos_current = robot->position(control_link, control_point);
			 cout << "INITIAL_APPROACH1 | Current:  "
				 << ee_pos_current.transpose()
				 << "  Desired:  " << ee_pos_desired.transpose() 
				 << "  Err: " << (ee_pos_current - ee_pos_desired).norm()
				 << endl;
 
			 const double thresh = 1e-2;  // 1 cm on XY maybe?
			 if ((ee_pos_current - ee_pos_desired).norm() < thresh) {
				pose_task -> disableVelocitySaturation();
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();
				prev_state = state;
				state = STOP;
				
			 }
		 }

		 else if (state == STOP) {
			if (clean1_start_time < 0.0) {
				clean1_start_time = time;
			}
			double t_elapsed = time - clean1_start_time;

			Vector3d ee_curr = robot->position(control_link, control_point);
			pose_task->setGoalPosition(ee_curr);

			// Build task and compute torques to hold pose
			N_prec.setIdentity();
			pose_task->updateTaskModel(N_prec);
			joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());

			command_torques = pose_task->computeTorques() + joint_task->computeTorques();

			const double thresh = 1e-2;  // 1 cm on XY maybe?
			if (t_elapsed > 2.0) {
				cout << "STOP, moving to next state\n";
				state = INITIAL_APPROACH2;
				// reset tasks here
				clean1_start_time = -1.0;
				pose_task->reInitializeTask();
				joint_task->reInitializeTask();
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