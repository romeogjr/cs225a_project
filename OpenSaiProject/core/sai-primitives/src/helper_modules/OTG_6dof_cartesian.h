/**
 * OTG_6dof_cartesian.h
 *
 *	A wrapper to use the Ruckig OTG library
 *	specifically to work for 6DOF position and orientation
 *
 * Author: Mikael Jorda
 * Created: August 2023
 */

#ifndef SAI_PRIMITIVES_OTG_6DOF_CARTESIAN_H
#define SAI_PRIMITIVES_OTG_6DOF_CARTESIAN_H

#include <Eigen/Dense>
#include <memory>
#include <ruckig/ruckig.hpp>

using namespace Eigen;
using namespace ruckig;
namespace SaiPrimitives {

typedef Matrix<double, 6, 1> Vector6d;

class OTG_6dof_cartesian {
public:
	/**
	 * @brief      constructor
	 *
	 * @param[in]  initial_position     The initial position
	 * @param[in]  initial_orientation  The initial orientation to initialize
	 *                                  the trajectory generation
	 * @param[in]  loop_time            The duration of a control loop
	 *                                  (typically, 0.001 if the robot is
	 *                                  controlled at 1 kHz)
	 */
	OTG_6dof_cartesian(const Vector3d& initial_position,
					   const Matrix3d& initial_orientation,
					   const double loop_time);

	/**
	 * @brief      destructor
	 */
	~OTG_6dof_cartesian() = default;

	/**
	 * @brief      Re initializes the trajectory generator so that the goal
	 * state is the state given as argument
	 *
	 * @param[in]  initial_position     The initial position
	 * @param[in]  initial_orientation  The initial orientation
	 */
	void reInitialize(const Vector3d& initial_position,
					  const Matrix3d& initial_orientation);

	/**
	 * @brief re initializes the linear part of the trajectory generator so that
	 * the goal state is the state given as argument
	 *
	 * @param initial_position The initial position
	 */
	void reInitializeLinear(const Vector3d& initial_position);
	/**
	 * @brief re initializes the angular part of the trajectory generator so
	 * that the goal state is the state given as argumentF
	 *
	 * @param initial_orientation The initial orientation
	 */
	void reInitializeAngular(const Matrix3d& initial_orientation);

	/**
	 * @brief      Sets the maximum linear velocity for the trajectory generator
	 *
	 * @param[in]  max_linear_velocity  Vector of the maximum velocity per
	 * direction
	 */
	void setMaxLinearVelocity(const Vector3d& max_linear_velocity);

	/**
	 * @brief      Sets the maximum linear velocity.
	 *
	 * @param[in]  max_linear_velocity  Scalar of the maximum velocity in all
	 * directions
	 */
	void setMaxLinearVelocity(const double max_linear_velocity) {
		setMaxLinearVelocity(max_linear_velocity * Vector3d::Ones());
	}

	/// @brief Getter for the maximum linear velocity
	Vector3d getMaxLinearVelocity() const {
		return _input.max_velocity.head<3>();
	}

	/**
	 * @brief      Sets the maximum linear acceleration.
	 *
	 * @param[in]  max_linear_acceleration  Vector of the maximum acceleration
	 */
	void setMaxLinearAcceleration(const Vector3d& max_linear_acceleration);

	/**
	 * @brief      Sets the maximum linear acceleration.
	 *
	 * @param[in]  max_linear_acceleration  Scalar of the maximum acceleration
	 */
	void setMaxLinearAcceleration(const double max_linear_acceleration) {
		setMaxLinearAcceleration(max_linear_acceleration * Vector3d::Ones());
	}

	/// @brief getter for the maximum linear acceleration
	Vector3d getMaxLinearAcceleration() const {
		return _input.max_acceleration.head<3>();
	}

	/**
	 * @brief      Sets the maximum angular velocity for the trajectory
	 * generator
	 *
	 * @param[in]  max_angular_velocity  Vector of the maximum velocity per
	 * direction
	 */
	void setMaxAngularVelocity(const Vector3d& max_angular_velocity);

	/**
	 * @brief      Sets the maximum angular velocity.
	 *
	 * @param[in]  max_angular_velocity  Scalar of the maximum velocity in all
	 * directions
	 */
	void setMaxAngularVelocity(const double max_angular_velocity) {
		setMaxAngularVelocity(max_angular_velocity * Vector3d::Ones());
	}

	/// @brief getter for the maximum angular velocity
	Vector3d getMaxAngularVelocity() const {
		return _input.max_velocity.tail<3>();
	}

	/**
	 * @brief      Sets the maximum angular acceleration.
	 *
	 * @param[in]  max_angular_acceleration  Vector of the maximum acceleration
	 */
	void setMaxAngularAcceleration(const Vector3d& max_angular_acceleration);

	/**
	 * @brief      Sets the maximum angular acceleration.
	 *
	 * @param[in]  max_angular_acceleration  Scalar of the maximum acceleration
	 */
	void setMaxAngularAcceleration(const double max_angular_acceleration) {
		setMaxAngularAcceleration(max_angular_acceleration * Vector3d::Ones());
	}

	/// @brief getter for the maximum angular acceleration
	Vector3d getMaxAngularAcceleration() const {
		return _input.max_acceleration.tail<3>();
	}

	/**
	 * @brief      Sets the maximum linear and angular jerk.
	 *
	 * @param[in]  max_angular_jerk  Vector of the maximum jerk
	 */
	void setMaxJerk(const Vector3d& max_linear_jerk,
					const Vector3d& max_angular_jerk);

	/**
	 * @brief      Sets the maximum linear and angular jerk.
	 *
	 * @param[in]  max_angular_jerk  Scalar of the maximum jerk
	 */
	void setMaxJerk(const double max_linear_jerk,
					const double max_angular_jerk) {
		setMaxJerk(max_linear_jerk * Vector3d::Ones(),
				   max_angular_jerk * Vector3d::Ones());
	}

	/// @brief getter for the maximum linear jerk
	Vector3d getMaxLinearJerk() const { return _input.max_jerk.head<3>(); }
	/// @brief getter for the maximum angular jerk
	Vector3d getMaxAngularJerk() const { return _input.max_jerk.tail<3>(); }

	/**
	 * @brief      Disables jerk limitation for the trajectory generator (enable
	 * them by setting jerk limits with the setMaxJerk function)
	 */
	void disableJerkLimits() {
		_input.max_jerk.setConstant(std::numeric_limits<double>::infinity());
	}

	/// @brief getter to know if jerk limits are enabled
	bool getJerkLimitEnabled() const {
		return _input.max_jerk !=
			   Vector6d::Constant(std::numeric_limits<double>::infinity());
	}

	/**
	 * @brief      Sets the goal position and velocity
	 *
	 * @param[in]  goal_position  The goal position
	 * @param[in]  goal_velocity  The goal velocity
	 */
	void setGoalPositionAndLinearVelocity(const Vector3d& goal_position,
										  const Vector3d& goal_linear_velocity);

	/**
	 * @brief      Sets the goal position with zero goal velocity
	 *
	 * @param[in]  goal_position  The goal position
	 */
	void setGoalPosition(const Vector3d& goal_position) {
		setGoalPositionAndLinearVelocity(goal_position, Vector3d::Zero());
	}

	/**
	 * @brief      Sets the goal orientation and angular velocity
	 *
	 * @param[in]  goal_orientation     The goal orientation
	 * @param[in]  current_orientation  The current orientation
	 * @param[in]  goal_velocity        The goal velocity
	 */
	void setGoalOrientationAndAngularVelocity(
		const Matrix3d& goal_orientation,
		const Vector3d& goal_angular_velocity);

	/**
	 * @brief      Sets the goal orientation with zero goal angular velocity
	 *
	 * @param[in]  goal_orientation     The goal orientation
	 * @param[in]  current_orientation  The current orientation
	 */
	void setGoalOrientation(const Matrix3d& goal_orientation) {
		setGoalOrientationAndAngularVelocity(goal_orientation,
											 Vector3d::Zero());
	}

	/**
	 * @brief      Runs the trajectory generation to compute the next desired
	 * state. Should be called once per control loop
	 *
	 */
	void update();

	/// @brief getter for the next position, output of the trajectory generator
	Vector3d getNextPosition() const { return _output.new_position.head<3>(); }
	/// @brief getter for the next linear velocity, output of the trajectory generator
	Vector3d getNextLinearVelocity() const {
		return _output.new_velocity.head<3>();
	}
	/// @brief getter for the next linear acceleration, output of the trajectory generator
	Vector3d getNextLinearAcceleration() const {
		return _output.new_acceleration.head<3>();
	}
	/// @brief getter for the next orientation, output of the trajectory generator
	Matrix3d getNextOrientation() const;
	/// @brief getter for the next angular velocity, output of the trajectory generator
	Vector3d getNextAngularVelocity() const {
		return _reference_frame * _output.new_velocity.tail<3>();
	}
	/// @brief getter for the next angular acceleration, output of the trajectory generator
	Vector3d getNextAngularAcceleration() const {
		return _reference_frame * _output.new_acceleration.tail<3>();
	}

	/**
	 * @brief      Function to know if the goal position and velocity is reached
	 *
	 * @return     true if the goal state is reached, false otherwise
	 */
	bool isGoalReached() const { return _goal_reached; }

private:
	/// @brief flag to know if the goal state is reached
	bool _goal_reached = false;
	/// @brief latest result of the ruckig trajectory generator
	int _result_value = Result::Finished;

	/// @brief the reference frame (initial orientation when the trajectory generator was initialized)
	Matrix3d _reference_frame;
	/// @brief the goal orientation in the base frame
	Matrix3d _goal_orientation_in_base_frame;
	/// @brief the goal angular velocity in the base frame
	Vector3d _goal_angular_velocity_in_base_frame;

	/// @brief the ruckig trajectory generator
	std::shared_ptr<Ruckig<6, EigenVector>> _otg;
	/// @brief the input parameters for the ruckig trajectory generator
	InputParameter<6, EigenVector> _input;
	/// @brief the output parameters for the ruckig trajectory generator
	OutputParameter<6, EigenVector> _output;
};

} /* namespace SaiPrimitives */

#endif	// SAI_PRIMITIVES_OTG_6DOF_CARTESIAN_H