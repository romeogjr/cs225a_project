/**
 * @file redis_keys.h
 * @author William Chong (williamchong@stanford.edu)
 * @brief 
 * @version 0.1
 * @date 2022-04-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai::sim::toro::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai::sim::toro::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai::sim::toro::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai::sim::toro::controller";
