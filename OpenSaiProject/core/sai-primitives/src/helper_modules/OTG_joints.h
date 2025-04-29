/**
 * OTG_joints.h
 *
 *	A wrapper to use the Ruckig OTG library
 *
 * Author: Mikael Jorda
 * Created: August 2023
 */

#ifndef SAI_PRIMITIVES_OTG_JOINTS_H
#define SAI_PRIMITIVES_OTG_JOINTS_H

#include <Eigen/Dense>
#include <ruckig/ruckig.hpp>

#include <memory>

using namespace Eigen;
using namespace ruckig;

namespace SaiPrimitives {

class OTG_joints {
public:
	/**
	 * @brief      constructor
	 *
	 * @param[in]  initial_position   Initial joint position. Serves to
	 * initialize the dimension of the space
	 * @param[in]  loop_time          The duration of a control loop (typically,
	 * 0.001 if the robot is controlled at 1 kHz)
	 */
	OTG_joints(const VectorXd& initial_position, const double loop_time);

	/**
	 * @brief      destructor
	 */
	~OTG_joints() = default;

	/**
	 * @brief 	Reinitializes the OTG_joints with a new initial position
	 *
	 * @param initial_position
	 */
	void reInitialize(const VectorXd& initial_position);

	/**
	 * @brief      Sets the maximum velocity for the trajectory generator
	 *
	 * @param[in]  max_velocity  Vector of the maximum velocity per direction
	 */
	void setMaxVelocity(const VectorXd& max_velocity);

	/**
	 * @brief      Sets the maximum velocity.
	 *
	 * @param[in]  max_velocity  Scalar of the maximum velocity in all
	 * directions
	 */
	void setMaxVelocity(const double max_velocity) {
		setMaxVelocity(max_velocity * VectorXd::Ones(_dim));
	}

	/// @brief getter for the maximum velocity
	VectorXd getMaxVelocity() const { return _input.max_velocity; }

	/**
	 * @brief      Sets the maximum acceleration.
	 *
	 * @param[in]  max_acceleration  Vector of the maximum acceleration
	 */
	void setMaxAcceleration(const VectorXd& max_acceleration);

	/**
	 * @brief      Sets the maximum acceleration.
	 *
	 * @param[in]  max_acceleration  Scalar of the maximum acceleration
	 */
	void setMaxAcceleration(const double max_acceleration) {
		setMaxAcceleration(max_acceleration * VectorXd::Ones(_dim));
	}

	/// @brief getter for the maximum acceleration
	VectorXd getMaxAcceleration() const { return _input.max_acceleration; }

	/**
	 * @brief      Sets the maximum jerk and enables jerk limitation for the
	 * trajectory generator
	 *
	 * @param[in]  max_jerk  Vector of the maximum jerk
	 */
	void setMaxJerk(const VectorXd& max_jerk);

	/**
	 * @brief      Sets the maximum jerk and enables jerk limitation for the
	 * trajectory generator
	 *
	 * @param[in]  max_jerk  Scalar of the maximum jerk
	 */
	void setMaxJerk(const double max_jerk) {
		setMaxJerk(max_jerk * VectorXd::Ones(_dim));
	}

	/// @brief getter for the maximum jerk
	VectorXd getMaxJerk() const { return _input.max_jerk; }

	/**
	 * @brief      Disables jerk limitation for the trajectory generator (enable
	 * them by setting jerk limits with the setMaxJerk function)
	 */
	void disableJerkLimits();

	/// @brief getter for the jerk limit enabled
	bool getJerkLimitEnabled() const;

	/**
	 * @brief      Sets the goal position and velocity
	 *
	 * @param[in]  goal_position  The goal position
	 * @param[in]  goal_velocity  The goal velocity
	 */
	void setGoalPositionAndVelocity(const VectorXd& goal_position,
									const VectorXd& goal_velocity);

	/**
	 * @brief      Sets the goal position and zero target velocity
	 *
	 * @param[in]  goal_position  The goal position
	 */
	void setGoalPosition(const VectorXd& goal_position) {
		setGoalPositionAndVelocity(goal_position, VectorXd::Zero(_dim));
	}

	/**
	 * @brief      Runs the trajectory generation to compute the next desired
	 * state. Should be called once per control loop
	 *
	 */
	void update();

	/**
	 * @brief      Gets the next position.
	 *
	 * @return     The next position.
	 */
	VectorXd getNextPosition() { return _output.new_position;}

	/**
	 * @brief      Gets the next velocity.
	 *
	 * @return     The next velocity.
	 */
	VectorXd getNextVelocity() { return _output.new_velocity;}

	/**
	 * @brief      Gets the next acceleration.
	 *
	 * @return     The next acceleration.
	 */
	VectorXd getNextAcceleration() { return _output.new_acceleration;}

	/**
	 * @brief      Function to know if the goal position and velocity is
	 reached
	 *
	 * @return     true if the goal state is reached, false otherwise
	 */
	bool isGoalReached() const { return _goal_reached; }

private:
	/// @brief dimension of the joint space
	int _dim;
	/// @brief flag to know if the goal state is reached
	bool _goal_reached = false;
	/// @brief latest result of the ruckig trajectory generator
	int _result_value = Result::Finished;

	/// @brief the goal position as an Eigen vector
	VectorXd _goal_position_eigen;
	/// @brief the goal velocity as an Eigen vector
	VectorXd _goal_velocity_eigen;

	/// @brief the ruckig trajectory generator
	std::unique_ptr<Ruckig<DynamicDOFs, EigenVector>> _otg;
	/// @brief the input parameter for the ruckig trajectory generator
	InputParameter<DynamicDOFs, EigenVector> _input {0};
	/// @brief the output parameter for the ruckig trajectory generator
	OutputParameter<DynamicDOFs, EigenVector> _output {0};
};

} /* namespace SaiPrimitives */

#endif	// SAI_PRIMITIVES_OTG_JOINTS_H