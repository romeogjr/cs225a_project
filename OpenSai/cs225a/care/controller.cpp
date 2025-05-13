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
	 CLEAN_1, 
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
 
			 // 1) set your desired goal
			 Vector3d ee_pos_desired;
			 ee_pos_desired << 0.5, 0.175, 0.6;
			 pose_task->setGoalPosition(ee_pos_desired);
 
			 // 2) turn on velocity saturation (linear , angular )
			 pose_task->enableVelocitySaturation(0.1, 0.5);
 
			 // 3) build your task hierarchy as usual
			 N_prec.setIdentity();
			 pose_task->updateTaskModel(N_prec);
			 joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
 
			 // 4) compute torques (now with sat’d vels)
			 command_torques = pose_task->computeTorques()
							 + joint_task->computeTorques();
 
			 // Print position				
			 Vector3d ee_pos_current = robot->position(control_link, control_point);
			 cout << "INITIAL_APPROACH | Current:  "
				 << ee_pos_current.transpose()
				 << "  Desired:  " << ee_pos_desired.transpose() 
				 << "  Err: " << (ee_pos_current - ee_pos_desired).norm()
				 << endl;
 
			 const double thresh = 1e-2;  // 1 cm on XY maybe?
			 if ((ee_pos_current - ee_pos_desired).norm() < thresh) {
				 // 1) change the Z‐goal to the exact torso center:
				 Vector3d center = ee_pos_current;
				 center.z() = 0.6;  
				 pose_task->setGoalPosition(center);
				 // 2) wait until Z is within tiny tol of 0.6
				 if (fabs(ee_pos_current.z() - 0.6) < 1e-3) {
					 cout << "Aligned at center height—starting CLEAN_1\n";
					 state = CLEAN_1;
					 // reset tasks here
					 clean1_start_time = -1.0;
					 pose_task->reInitializeTask();
					 joint_task->reInitializeTask();
				 }
			 }
	 
 
		 } else if (state == CLEAN_1) {
			 // initialize start time
			 if (clean1_start_time < 0.0) {
				 clean1_start_time = time;
			 }
			 double t_elapsed = time - clean1_start_time;
 
			 // ■ torso & sponge geometry (from your URDF)
			 const double torso_center_z    = 0.6;     // from <origin xyz=…>
			 const double torso_half_height = 0.5 / 2; // box size z = 0.5
			 const double sponge_length     = 0.03;    // cylinder length
			 const double sponge_half_len   = sponge_length / 2;
 
			 // ■ compute amplitude so sponge never crosses top/bottom
			 double amplitude_z = torso_half_height - sponge_half_len;
 
			 // ■ pick a frequency (e.g. 0.2 Hz ⇒ one full up+down every 5 s)
			 const double freq  = 0.2;
			 double omega = 2.0 * M_PI * freq;
 
			 // ■ desired Z = center + amplitude * sin(ω t)
			 double z_des = torso_center_z + amplitude_z * sin(omega * t_elapsed);
 
			 // ■ build the full 3D goal (keep your contact X/Y fixed)
			 Vector3d ee_cur = robot->position(control_link, control_point);
			 Vector3d goal;
			 goal << ee_cur.x(), ee_cur.y(), z_des;
			 pose_task->setGoalPosition(goal);
			 pose_task->enableVelocitySaturation(0.1, 0.5);
 
			 // ■ task hierarchy + torque
			 N_prec.setIdentity();
			 pose_task->updateTaskModel(N_prec);
			 joint_task->updateTaskModel(pose_task->getTaskAndPreviousNullspace());
			 command_torques = pose_task->computeTorques()
							 + joint_task->computeTorques();
 
			 // ■ debug print
			 cout << "CLEAN_1 | z_cur=" << ee_cur.z()
				 << " z_des=" << z_des
				 << " err=" << fabs(ee_cur.z()-z_des) << endl;
 
			 // ■ after 15 s, go to RETRACT
			 if (t_elapsed > 15.0) {
				 cout << "CLEAN_1 complete, switching to RETRACT\n";
				 state = RETRACT;
				 pose_task->reInitializeTask();
				 joint_task->reInitializeTask();
			 }
		 
 
 
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