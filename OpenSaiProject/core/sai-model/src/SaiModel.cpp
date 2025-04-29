/*
 * SaiModel.cpp
 *
 *  Wrapper around RBDL plus functions to facilitate the whole body control
 * framework from Stanford robotics lab
 *
 *  Created on: Dec 14, 2016
 *      Author: Mikael Jorda
 */

#include "SaiModel.h"

#include <UrdfToSaiModel.h>

#include "RBDLExtensions.h"

using namespace std;
using namespace Eigen;

namespace {

bool isValidQuaternion(double x, double y, double z, double w) {
	Eigen::Quaterniond quaternion(w, x, y, z);

	if (std::abs(quaternion.squaredNorm() - 1.0) > 1e-3) {
		return false;
	}

	return true;
}

bool isPositiveDefinite(const MatrixXd& matrix) {
	// square
	if (matrix.rows() != matrix.cols()) return false;

	// symmetric
	if (!matrix.transpose().isApprox(matrix, 1e-2)) return false;

	// positive eigenvalues
	SelfAdjointEigenSolver<MatrixXd> eigensolver(matrix);
	const auto& eigenvalues = eigensolver.eigenvalues();

	for (int i = 0; i < eigenvalues.size(); ++i) {
		if (eigenvalues(i) <= 0) return false;
	}

	return true;
}

}  // namespace

namespace SaiModel {

SaiModel::SaiModel(const string path_to_model_file, bool verbose) {
	_rbdl_model = new RigidBodyDynamics::Model();

	// parse rbdl model from urdf
	map<int, double> initial_joint_positions;
	bool success = RigidBodyDynamics::URDFReadFromFile(
		path_to_model_file.c_str(), _rbdl_model, _link_names_to_id_map,
		_joint_names_to_id_map, initial_joint_positions,
		_joint_names_to_child_link_names_map,
		_joint_names_to_parent_link_names_map, _joint_limits, false, verbose);
	if (!success) {
		throw std::runtime_error("Error loading model [" + path_to_model_file +
								 "]\n");
	}

	// create joint id to name map
	for (auto pair : _joint_names_to_id_map) {
		string joint_name = pair.first;
		int rbdl_index = pair.second;
		const auto& rbdl_joint = _rbdl_model->mJoints[rbdl_index];
		switch (rbdl_joint.mJointType) {
			case RigidBodyDynamics::JointTypeSpherical:
				for (int i = 0; i < 3; ++i) {
					_joint_id_to_names_map[rbdl_joint.q_index + i] = joint_name;
				}
				_joint_id_to_names_map[_rbdl_model
										   ->multdof3_w_index[rbdl_index]] =
					joint_name;
				break;

			case RigidBodyDynamics::JointTypeRevolute:
			case RigidBodyDynamics::JointTypeRevoluteX:
			case RigidBodyDynamics::JointTypeRevoluteY:
			case RigidBodyDynamics::JointTypeRevoluteZ:
			case RigidBodyDynamics::JointTypePrismatic:
				_joint_id_to_names_map[rbdl_joint.q_index] = joint_name;
				break;

			default:
				throw std::runtime_error(
					"error generating joint id to name map\n");
				break;
		}
	}

	// set the base position in the world
	_T_world_robot = Affine3d::Identity();

	// set the gravity in rbdl model
	// it is the world gravity expressed in robot base frame
	_rbdl_model->gravity = Vector3d(0, 0, -9.81);

	// set the number of degrees of freedom
	_dof = _rbdl_model->dof_count;

	// set the size of q vector
	_q_size = _rbdl_model->q_size;

	// Initialize state vectors
	_q.setZero(_q_size);
	for (const auto& pair : initial_joint_positions) {
		_q(pair.first) = pair.second;
	}
	// special case handle spherical joints. See rbdl/Joint class for details.
	for (uint i = 0; i < _rbdl_model->mJoints.size(); ++i) {
		if (_rbdl_model->mJoints[i].mJointType ==
			RigidBodyDynamics::JointTypeSpherical) {
			_rbdl_model->SetQuaternion(i, RigidBodyDynamics::Math::Quaternion(),
									   _q);
			int index = _rbdl_model->mJoints[i].q_index;
			int w_index = _rbdl_model->multdof3_w_index[i];
			string joint_name = jointName(index);
			_spherical_joints.push_back(SphericalJointDescription(
				joint_name, parentLinkName(joint_name),
				childLinkName(joint_name), index, w_index));
		}
	}

	_dq.setZero(_dof);
	_ddq.setZero(_dof);
	_M.setIdentity(_dof, _dof);
	_M_inv.setIdentity(_dof, _dof);

	updateModel();
}

SaiModel::~SaiModel() {
	delete _rbdl_model;
	_rbdl_model = NULL;
}

void SaiModel::setQ(const Eigen::VectorXd& q) {
	if (q.size() != _q_size) {
		throw invalid_argument("q size inconsistent in SaiModel::setQ");
		return;
	}
	for (const auto& sph_joint : _spherical_joints) {
		if (!isValidQuaternion(q(sph_joint.index), q(sph_joint.index + 1),
							   q(sph_joint.index + 2), q(sph_joint.w_index))) {
			throw invalid_argument(
				"trying to set an invalid quaternion for joint " +
				sph_joint.joint_name + " at index " +
				std::to_string(sph_joint.index) +
				", and w_index: " + std::to_string(sph_joint.w_index));
			return;
		}
	}
	_q = q;
}

void SaiModel::setDq(const Eigen::VectorXd& dq) {
	if (dq.size() != _dof) {
		throw invalid_argument("dq size inconsistent in SaiModel::setDq");
		return;
	}
	_dq = dq;
}

void SaiModel::setDdq(const Eigen::VectorXd& ddq) {
	if (ddq.size() != _dof) {
		throw invalid_argument("ddq size inconsistent in SaiModel::setDdq");
		return;
	}
	_ddq = ddq;
}

VectorXd SaiModel::jointLimitsPositionLower() const {
	VectorXd lower_limits =
		-numeric_limits<double>::max() * VectorXd::Ones(_q_size);
	for (const auto& joint_limit : _joint_limits) {
		lower_limits(joint_limit.joint_index) = joint_limit.position_lower;
	}
	for (const auto& joint : _spherical_joints) {
		lower_limits(joint.index) = -1.0;
		lower_limits(joint.index + 1) = -1.0;
		lower_limits(joint.index + 2) = -1.0;
		lower_limits(joint.w_index) = -1.0;
	}
	return lower_limits;
}

VectorXd SaiModel::jointLimitsPositionUpper() const {
	VectorXd upper_limits =
		numeric_limits<double>::max() * VectorXd::Ones(_q_size);
	for (const auto& joint_limit : _joint_limits) {
		upper_limits(joint_limit.joint_index) = joint_limit.position_upper;
	}
	for (const auto& joint : _spherical_joints) {
		upper_limits(joint.index) = 1.0;
		upper_limits(joint.index + 1) = 1.0;
		upper_limits(joint.index + 2) = 1.0;
		upper_limits(joint.w_index) = 1.0;
	}
	return upper_limits;
}

const Eigen::Quaterniond SaiModel::sphericalQuat(
	const std::string& joint_name) const {
	for (auto joint : _spherical_joints) {
		if (joint.joint_name == joint_name) {
			int i = joint.index;
			int iw = joint.w_index;
			return Eigen::Quaterniond(_q(iw), _q(i), _q(i + 1), _q(i + 2));
		}
	}
	throw invalid_argument(
		"cannot get the quaternion for non existing spherical joint " +
		joint_name);
}

void SaiModel::setSphericalQuat(const std::string& joint_name,
								 const Eigen::Quaterniond quat) {
	for (auto joint : _spherical_joints) {
		if (joint.joint_name == joint_name) {
			int i = joint.index;
			int iw = joint.w_index;
			_q(i) = quat.x();
			_q(i + 1) = quat.y();
			_q(i + 2) = quat.z();
			_q(iw) = quat.w();
			return;
		}
	}
	throw invalid_argument(
		"cannot set the quaternion for non existing spherical joint " +
		joint_name);
}

void SaiModel::setTRobotBase(const Affine3d& T) {
	Vector3d world_gravity = worldGravity();
	_T_world_robot = T;
	// world gravity in robot base frame changes
	_rbdl_model->gravity = _T_world_robot.linear().transpose() * world_gravity;
}

bool SaiModel::isLinkInRobot(const std::string& link_name) const {
	if (_link_names_to_id_map.find(link_name) == _link_names_to_id_map.end()) {
		return false;
	}
	return true;
}

void SaiModel::updateKinematics() {
	UpdateKinematicsCustom(*_rbdl_model, &_q, &_dq, &_ddq);
}

void SaiModel::updateModel() {
	updateKinematics();
	updateDynamics();
}

void SaiModel::updateModel(const Eigen::MatrixXd& M) {
	updateKinematics();

	if (!isPositiveDefinite(M)) {
		throw invalid_argument(
			"M is not symmetric positive definite SaiModel::updateModel");
	}
	if (M.rows() != _dof) {
		throw invalid_argument(
			"M matrix dimensions inconsistent in SaiModel::updateModel");
	}
	_M = M;
	updateInverseInertia();
}

VectorXd SaiModel::jointGravityVector() {
	VectorXd g = VectorXd::Zero(_dof);

	vector<RigidBodyDynamics::Body>::iterator it_body;
	int body_id;

	for (it_body = _rbdl_model->mBodies.begin(), body_id = 0;
		 it_body != _rbdl_model->mBodies.end(); ++it_body, ++body_id) {
		double mass = it_body->mMass;
		MatrixXd Jv = MatrixXd::Zero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, body_id, it_body->mCenterOfMass, Jv,
						  false);

		g += Jv.transpose() * (-mass * _rbdl_model->gravity);
	}
	return g;
}

VectorXd SaiModel::coriolisForce() {
	// returns v + g. Need to substract the gravity from it
	VectorXd b = VectorXd::Zero(_dof);
	NonlinearEffects(*_rbdl_model, _q, _dq, b);

	return b - jointGravityVector();
}

VectorXd SaiModel::coriolisPlusGravity() {
	VectorXd h = VectorXd::Zero(_dof);
	NonlinearEffects(*_rbdl_model, _q, _dq, h);
	return h;
}

MatrixXd SaiModel::factorizedChristoffelMatrix() {
	MatrixXd C = MatrixXd::Zero(_dof, _dof);

	VectorXd vi = VectorXd::Zero(_dof);

	for (int i = 0; i < _dof; i++) {
		vi.setZero();
		vi(i) = 1;
		C.col(i) =
			modifiedNewtonEuler(false, _q, _dq, vi, VectorXd::Zero(_dof));
	}
	return C;
}

MatrixXd SaiModel::J(const string& link_name,
					  const Vector3d& pos_in_link) const {
	MatrixXd J = MatrixXd::Zero(6, _dof);
	CalcPointJacobian6D(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
						false);
	// RBDL gives Jw as the top 3 rows and Jv as the bottom part. We need to
	// swap it here
	J.block(0, 0, 3, _dof).swap(J.block(3, 0, 3, _dof));
	return J;
}

MatrixXd SaiModel::JWorldFrame(const string& link_name,
								const Vector3d& pos_in_link) const {
	MatrixXd J = this->J(link_name, pos_in_link);
	J.topRows<3>() = _T_world_robot.linear() * J.topRows<3>();
	J.bottomRows<3>() = _T_world_robot.linear() * J.bottomRows<3>();
	return J;
}

MatrixXd SaiModel::JLocalFrame(const string& link_name,
								const Vector3d& pos_in_link,
								const Matrix3d& rot_in_link) const {
	MatrixXd J = this->J(link_name, pos_in_link);
	Matrix3d R_base_ee = rotation(link_name) * rot_in_link;
	J.topRows<3>() = R_base_ee.transpose() * J.topRows<3>();
	J.bottomRows<3>() = R_base_ee.transpose() * J.bottomRows<3>();
	return J;
}

MatrixXd SaiModel::Jv(const string& link_name,
					   const Vector3d& pos_in_link) const {
	MatrixXd J = MatrixXd::Zero(3, _dof);
	CalcPointJacobian(*_rbdl_model, _q, linkIdRbdl(link_name), pos_in_link, J,
					  false);
	return J;
}

MatrixXd SaiModel::JvWorldFrame(const string& link_name,
								 const Vector3d& pos_in_link) const {
	return _T_world_robot.linear() * Jv(link_name, pos_in_link);
}

MatrixXd SaiModel::JvLocalFrame(const string& link_name,
								 const Vector3d& pos_in_link,
								 const Matrix3d& rot_in_link) const {
	return rotation(link_name, rot_in_link).transpose() *
		   Jv(link_name, pos_in_link);
}

MatrixXd SaiModel::Jw(const string& link_name) const {
	// compute the full jacobian at the center of the link and take rotational
	// part
	return J(link_name).bottomRows<3>();
}

MatrixXd SaiModel::JwWorldFrame(const string& link_name) const {
	return _T_world_robot.linear() * Jw(link_name);
}

MatrixXd SaiModel::JwLocalFrame(const string& link_name,
								 const Matrix3d& rot_in_link) const {
	return rotation(link_name, rot_in_link).transpose() * Jw(link_name);
}

VectorXd SaiModel::computeInverseKinematics(
	const vector<string>& link_names, const vector<Vector3d>& pos_in_links,
	const vector<Vector3d>& desired_pos_world_frame) {
	if (link_names.size() != pos_in_links.size() ||
		link_names.size() != desired_pos_world_frame.size()) {
		throw runtime_error(
			"the number of link names, pos in links and desired positions must "
			"match in SaiModel::computeInverseKinematics\n");
	}
	if (link_names.empty()) {
		cout << "Warning: trying to compute inverse kinematics with no goal. "
				"returning current model joint positions"
			 << endl;
		return _q;
	}

	RigidBodyDynamics::InverseKinematicsConstraintSet cs;
	for (int i = 0; i < link_names.size(); i++) {
		cs.AddPointConstraint(
			linkIdRbdl(link_names[i]), pos_in_links[i],
			_T_world_robot.inverse() * desired_pos_world_frame[i]);
	}

	return IKInternal(cs);
}

VectorXd SaiModel::computeInverseKinematics(
	const vector<string>& link_names, const vector<Affine3d>& frames_in_links,
	const vector<Affine3d>& desired_frames_locations_in_world_frame) {
	if (link_names.size() != frames_in_links.size() ||
		link_names.size() != desired_frames_locations_in_world_frame.size()) {
		throw runtime_error(
			"the number of link names, pos in links and desired positions must "
			"match in SaiModel::computeInverseKinematics\n");
	}
	if (link_names.empty()) {
		cout << "Warning: trying to compute inverse kinematics with no goal. "
				"returning current model joint positions"
			 << endl;
		return _q;
	}

	RigidBodyDynamics::InverseKinematicsConstraintSet cs;
	for (int i = 0; i < link_names.size(); i++) {
		Affine3d robot_base_to_desired_link_base_frame =
			_T_world_robot.inverse() *
			desired_frames_locations_in_world_frame[i] *
			frames_in_links[i].inverse();

		cs.AddFullConstraint(
			linkIdRbdl(link_names[i]), Vector3d::Zero(),
			robot_base_to_desired_link_base_frame.translation(),
			robot_base_to_desired_link_base_frame.rotation().transpose());
	}

	return IKInternal(cs);
}

VectorXd SaiModel::IKInternal(
	RigidBodyDynamics::InverseKinematicsConstraintSet& cs) {
	cs.step_tol = 1e-9;
	cs.constraint_tol = 1e-6;
	cs.lambda = 0.1;

	VectorXd q_min =
		std::numeric_limits<double>::min() * VectorXd::Ones(_q_size);
	VectorXd q_max =
		std::numeric_limits<double>::max() * VectorXd::Ones(_q_size);
	for (const auto& joint_limit : _joint_limits) {
		q_min(joint_limit.joint_index) = joint_limit.position_lower;
		q_max(joint_limit.joint_index) = joint_limit.position_upper;
	}

	VectorXd q_res = VectorXd::Zero(_q_size);
	InverseKinematicsWithJointLimits(*_rbdl_model, _q, cs, q_min, q_max, q_res);

	// InverseKinematicsWithJointLimits modifies the internal model so we need
	// to re update the kinematics with the previous q value to keep it
	// unchanged
	updateKinematics();

	return q_res;
}

Affine3d SaiModel::transform(const string& link_name,
							  const Vector3d& pos_in_link,
							  const Matrix3d& rot_in_link) const {
	unsigned int link_id = linkIdRbdl(link_name);
	Eigen::Affine3d T(
		CalcBodyWorldOrientation(*_rbdl_model, _q, link_id, false).transpose() *
		rot_in_link);
	T.translation() = CalcBodyToBaseCoordinates(*_rbdl_model, _q, link_id,
												pos_in_link, false);
	return T;
}

Affine3d SaiModel::transformInWorld(const string& link_name,
									 const Vector3d& pos_in_link,
									 const Matrix3d& rot_in_link) const {
	return _T_world_robot * transform(link_name, pos_in_link, rot_in_link);
}

Vector6d SaiModel::velocity6d(const string link_name,
							   const Vector3d& pos_in_link) const {
	Vector6d vel6d = CalcPointVelocity6D(
		*_rbdl_model, _q, _dq, linkIdRbdl(link_name), pos_in_link, false);
	vel6d.head(3).swap(vel6d.tail(3));
	return vel6d;
}

Vector6d SaiModel::velocity6dInWorld(const string link_name,
									  const Vector3d& pos_in_link) const {
	Vector6d vel6d = velocity6d(link_name, pos_in_link);
	vel6d.head(3) = _T_world_robot.linear() * vel6d.head(3);
	vel6d.tail(3) = _T_world_robot.linear() * vel6d.tail(3);
	return vel6d;
}

Vector6d SaiModel::acceleration6d(const string link_name,
								   const Vector3d& pos_in_link) const {
	// recompute accelerations only in rbdl model to make sure accelerations due
	// to gravity are not included in the return value
	UpdateKinematicsCustom(*_rbdl_model, nullptr, nullptr, &_ddq);
	Vector6d acc6d = CalcPointAcceleration6D(
		*_rbdl_model, _q, _dq, _ddq, linkIdRbdl(link_name), pos_in_link, false);
	acc6d.head(3).swap(acc6d.tail(3));
	return acc6d;
}
Vector6d SaiModel::acceleration6dInWorld(const string link_name,
										  const Vector3d& pos_in_link) const {
	Vector6d accel6d = acceleration6d(link_name, pos_in_link);
	accel6d.head(3) = _T_world_robot.linear() * accel6d.head(3);
	accel6d.tail(3) = _T_world_robot.linear() * accel6d.tail(3);
	return accel6d;
}

Vector3d SaiModel::position(const string& link_name,
							 const Vector3d& pos_in_link) const {
	return CalcBodyToBaseCoordinates(*_rbdl_model, _q, linkIdRbdl(link_name),
									 pos_in_link, false);
}

Vector3d SaiModel::positionInWorld(const string& link_name,
									const Vector3d& pos_in_link) const {
	return _T_world_robot * position(link_name, pos_in_link);
}

Vector3d SaiModel::linearVelocity(const string& link_name,
								   const Vector3d& pos_in_link) const {
	return CalcPointVelocity(*_rbdl_model, _q, _dq, linkIdRbdl(link_name),
							 pos_in_link, false);
}
Vector3d SaiModel::linearVelocityInWorld(const string& link_name,
										  const Vector3d& pos_in_link) const {
	return _T_world_robot.linear() * linearVelocity(link_name, pos_in_link);
}

Vector3d SaiModel::linearAcceleration(const string& link_name,
									   const Vector3d& pos_in_link) const {
	// recompute accelerations only in rbdl model to make sure accelerations due
	// to gravity are not included in the return value
	UpdateKinematicsCustom(*_rbdl_model, nullptr, nullptr, &_ddq);
	return CalcPointAcceleration(*_rbdl_model, _q, _dq, _ddq,
								 linkIdRbdl(link_name), pos_in_link, false);
}
Vector3d SaiModel::linearAccelerationInWorld(
	const string& link_name, const Vector3d& pos_in_link) const {
	return _T_world_robot.linear() * linearAcceleration(link_name, pos_in_link);
}

Matrix3d SaiModel::rotation(const string& link_name,
							 const Matrix3d& rot_in_link) const {
	return CalcBodyWorldOrientation(*_rbdl_model, _q, linkIdRbdl(link_name),
									false)
			   .transpose() *
		   rot_in_link;
}

Matrix3d SaiModel::rotationInWorld(const string& link_name,
									const Matrix3d& rot_in_link) const {
	return _T_world_robot.linear() * rotation(link_name, rot_in_link);
}

Vector3d SaiModel::angularVelocity(const string& link_name) const {
	return velocity6d(link_name, Vector3d::Zero()).tail(3);
}

Vector3d SaiModel::angularVelocityInWorld(const string& link_name) const {
	return _T_world_robot.linear() * angularVelocity(link_name);
}

Vector3d SaiModel::angularAcceleration(const string& link_name) const {
	return acceleration6d(link_name, Vector3d::Zero()).tail(3);
}
Vector3d SaiModel::angularAccelerationInWorld(const string& link_name) const {
	return _T_world_robot.linear() * angularAcceleration(link_name);
}

unsigned int SaiModel::linkIdRbdl(const string& link_name) const {
	if (_link_names_to_id_map.find(link_name) == _link_names_to_id_map.end()) {
		throw invalid_argument("link [" + link_name + "] does not exist");
	}
	return _link_names_to_id_map.at(link_name);
}

int SaiModel::jointIndex(const string& joint_name) const {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw invalid_argument("joint [" + joint_name + "] does not exist");
	}
	return _rbdl_model->mJoints.at(_joint_names_to_id_map.at(joint_name))
		.q_index;
}

int SaiModel::sphericalJointIndexW(const string& joint_name) const {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw invalid_argument("joint [" + joint_name + "] does not exist");
	}
	for (auto it = _spherical_joints.cbegin(); it != _spherical_joints.cend();
		 ++it) {
		if (it->joint_name == joint_name) {
			return it->w_index;
		}
	}
	throw invalid_argument("joint [" + joint_name + "] is not spherical");
}

std::string SaiModel::jointName(const int joint_id) const {
	if (joint_id < 0 || joint_id >= _q_size) {
		throw std::invalid_argument(
			"cannot get joint name for id out of bounds");
	}
	return _joint_id_to_names_map.at(joint_id);
}

std::string SaiModel::childLinkName(const std::string& joint_name) const {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw invalid_argument("joint [" + joint_name + "] does not exist");
	}
	return _joint_names_to_child_link_names_map.at(joint_name);
}

std::string SaiModel::parentLinkName(const std::string& joint_name) const {
	if (_joint_names_to_id_map.find(joint_name) ==
		_joint_names_to_id_map.end()) {
		throw invalid_argument("joint [" + joint_name + "] does not exist");
	}
	return _joint_names_to_parent_link_names_map.at(joint_name);
}

std::vector<std::string> SaiModel::jointNames() const {
	std::vector<std::string> names;
	names.reserve(_joint_id_to_names_map.size());

	for (const auto& pair : _joint_id_to_names_map) {
		names.push_back(pair.second);
	}

	return names;
}

LinkMassParams SaiModel::getLinkMassParams(const string& link_name) const {
	RigidBodyDynamics::Body b = _rbdl_model->mBodies.at(linkIdRbdl(link_name));
	return LinkMassParams(b.mMass, b.mCenterOfMass, b.mInertia);
}

Vector3d SaiModel::comPosition() const {
	Vector3d robot_com = Vector3d::Zero();
	double robot_mass = 0.0;
	Vector3d center_of_mass_global_frame;
	int n_bodies = _rbdl_model->mBodies.size();
	for (int i = 0; i < n_bodies; i++) {
		RigidBodyDynamics::Body b = _rbdl_model->mBodies.at(i);

		center_of_mass_global_frame = CalcBodyToBaseCoordinates(
			*_rbdl_model, _q, i, b.mCenterOfMass, false);

		robot_com += center_of_mass_global_frame * b.mMass;
		robot_mass += b.mMass;
	}
	return robot_com / robot_mass;
}

MatrixXd SaiModel::comJacobian() const {
	MatrixXd Jv_com = MatrixXd::Zero(3, _dof);
	MatrixXd link_Jv;
	double robot_mass = 0.0;
	int n_bodies = _rbdl_model->mBodies.size();
	for (int i = 0; i < n_bodies; i++) {
		RigidBodyDynamics::Body b = _rbdl_model->mBodies[i];

		link_Jv.setZero(3, _dof);
		CalcPointJacobian(*_rbdl_model, _q, i, b.mCenterOfMass, link_Jv, false);

		Jv_com += link_Jv * b.mMass;
		robot_mass += b.mMass;
	}
	return Jv_com / robot_mass;
}

Eigen::MatrixXd SaiModel::taskInertiaMatrix(
	const MatrixXd& task_jacobian) const {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"SaiModel::taksInertiaMatrix");
	}

	// resize Matrices
	int k = task_jacobian.rows();

	// compute task inertia
	MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();
	return inv_inertia.llt().solve(MatrixXd::Identity(k, k));
}

MatrixXd SaiModel::taskInertiaMatrixWithPseudoInv(
	const MatrixXd& task_jacobian) const {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"SaiModel::taksInertiaMatrixWithPseudoInv");
	}

	// resize Matrices
	int k = task_jacobian.rows();

	// compute task inertia
	MatrixXd inv_inertia = task_jacobian * _M_inv * task_jacobian.transpose();

	// compute SVD pseudoinverse
	return computePseudoInverse(inv_inertia);
}

MatrixXd SaiModel::dynConsistentInverseJacobian(
	const MatrixXd& task_jacobian) const {
	// check the task jacobian is compatible with the robot model
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"SaiModel::dynConsistentInverseJacobian");
	}
	return _M_inv * task_jacobian.transpose() *
		   taskInertiaMatrix(task_jacobian);
}

MatrixXd SaiModel::nullspaceMatrix(const MatrixXd& task_jacobian) const {
	// check matrices dimmnsions
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"jacobian matrix dimensions inconsistent with model dof in "
			"SaiModel::nullspaceMatrix");
	}

	MatrixXd Jbar = dynConsistentInverseJacobian(task_jacobian);
	return (MatrixXd::Identity(_dof, _dof) - Jbar * task_jacobian);
}

OpSpaceMatrices SaiModel::operationalSpaceMatrices(
	const MatrixXd& task_jacobian) const {
	// check matrices have the right size
	if (task_jacobian.cols() != _dof) {
		throw invalid_argument(
			"Jacobian size inconsistent with DOF of robot model in "
			"SaiModel::operationalSpaceMatrices");
	}

	// Compute the matrices
	MatrixXd Lambda = taskInertiaMatrix(task_jacobian);
	MatrixXd Jbar = _M_inv * task_jacobian.transpose() * Lambda;
	MatrixXd N = (MatrixXd::Identity(_dof, _dof) - Jbar * task_jacobian);
	return OpSpaceMatrices(task_jacobian, Lambda, Jbar, N);
}

void SaiModel::addEnvironmentalContact(const string link,
										const Vector3d pos_in_link,
										const Matrix3d orientation,
										const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _environmental_contacts.begin();
		 it != _environmental_contacts.end(); ++it) {
		if (it->contact_link_name == link) {
			throw invalid_argument(
				"Environmental contact on link " + link +
				" already exists in SaiModel::addEnvironmentalContact()");
		}
	}
	_environmental_contacts.push_back(
		ContactModel(link, pos_in_link, orientation, contact_type));
}
void SaiModel::deleteEnvironmentalContact(const string link_name) {
	vector<ContactModel> new_contacts;
	for (vector<ContactModel>::iterator it = _environmental_contacts.begin();
		 it != _environmental_contacts.end(); ++it) {
		if (it->contact_link_name != link_name) {
			new_contacts.push_back(*it);
		}
	}
	_environmental_contacts = new_contacts;
}

void SaiModel::updateEnvironmentalContact(const string link,
										   const Vector3d pos_in_link,
										   const Matrix3d orientation,
										   const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _environmental_contacts.begin();
		 it != _environmental_contacts.end(); ++it) {
		if (it->contact_link_name == link) {
			it->contact_position = pos_in_link;
			it->contact_orientation = orientation;
			it->contact_type = contact_type;
			return;
		}
	}
	throw invalid_argument(
		"Environmental contact on link " + link +
		" does not exist in SaiModel::updateManipulationContact()");
}

void SaiModel::addManipulationContact(const string link,
									   const Vector3d pos_in_link,
									   const Matrix3d orientation,
									   const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _manipulation_contacts.begin();
		 it != _manipulation_contacts.end(); ++it) {
		if (it->contact_link_name == link) {
			throw invalid_argument(
				"Environmental contact on link " + link +
				" already exists in SaiModel::addManipulationContact()");
		}
	}
	_manipulation_contacts.push_back(
		ContactModel(link, pos_in_link, orientation, contact_type));
}

void SaiModel::deleteManipulationContact(const string link_name) {
	vector<ContactModel> new_contacts;
	for (vector<ContactModel>::iterator it = _manipulation_contacts.begin();
		 it != _manipulation_contacts.end(); ++it) {
		if (it->contact_link_name != link_name) {
			new_contacts.push_back(*it);
		}
	}
	_manipulation_contacts = new_contacts;
}

void SaiModel::updateManipulationContact(const string link,
										  const Vector3d pos_in_link,
										  const Matrix3d orientation,
										  const ContactType contact_type) {
	for (vector<ContactModel>::iterator it = _manipulation_contacts.begin();
		 it != _manipulation_contacts.end(); ++it) {
		if (it->contact_link_name == link) {
			it->contact_position = pos_in_link;
			it->contact_orientation = orientation;
			it->contact_type = contact_type;
			return;
		}
	}
	throw invalid_argument(
		"Environmental contact on link " + link +
		" does not exist in SaiModel::updateManipulationContact()");
}

GraspMatrixData SaiModel::manipulationGraspMatrix(
	const Vector3d& desired_resultant_point,
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	GraspMatrixData G_data = manipulationGraspMatrixAtGeometricCenter(
		resultant_in_world_frame, contact_forces_in_local_frames);

	Vector3d r_cg = G_data.resultant_point - desired_resultant_point;
	Matrix3d Rcross_cg = crossProductOperator(r_cg);

	int n = G_data.G.cols();

	G_data.G.block(3, 0, 3, n) += Rcross_cg * G_data.G.block(0, 0, 3, n);
	G_data.G_inv.block(0, 0, n, 3) -=
		G_data.G_inv.block(0, 3, n, 3) * Rcross_cg;

	G_data.resultant_point = desired_resultant_point;

	return G_data;
}

GraspMatrixData SaiModel::manipulationGraspMatrixAtGeometricCenter(
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	int num_surface_contacts = 0;
	for (const auto& contact : _manipulation_contacts) {
		if (resultant_in_world_frame) {
			contact_locations.push_back(
				positionInWorld(contact.contact_link_name, contact.contact_position));

		} else {
			contact_locations.push_back(
				position(contact.contact_link_name, contact.contact_position));
		}
		contact_types.push_back(contact.contact_type);
		if (contact.contact_type == SurfaceContact) {
			num_surface_contacts++;
		}
	}
	GraspMatrixData G_data =
		graspMatrixAtGeometricCenter(contact_locations, contact_types);

	if (contact_forces_in_local_frames) {
		int n_contact_points = _manipulation_contacts.size();
		int current_surface_contact = 0;
		for (int i = 0; i < n_contact_points; i++) {
			Matrix3d R_local;
			if (resultant_in_world_frame) {
				R_local = rotationInWorld(
					_manipulation_contacts[i].contact_link_name,
					_manipulation_contacts[i].contact_orientation);
			} else {
				R_local =
					rotation(_manipulation_contacts[i].contact_link_name,
							 _manipulation_contacts[i].contact_orientation);
			}

			G_data.G
				.block(0, 3 * i, 3 * (n_contact_points + num_surface_contacts),
					   3)
				.applyOnTheRight(R_local);
			G_data.G_inv
				.block(3 * i, 0, 3,
					   3 * (n_contact_points + num_surface_contacts))
				.applyOnTheLeft(R_local.transpose());

			if (_manipulation_contacts[i].contact_type == SurfaceContact) {
				G_data.G
					.block(0, 3 * (n_contact_points + current_surface_contact),
						   3 * (n_contact_points + num_surface_contacts), 3)
					.applyOnTheRight(R_local);
				G_data.G_inv
					.block(3 * (n_contact_points + current_surface_contact), 0,
						   3, 3 * (n_contact_points + num_surface_contacts))
					.applyOnTheLeft(R_local.transpose());

				if (n_contact_points > 2) {
					G_data.G
						.block(3 * (n_contact_points + current_surface_contact),
							   0, 3,
							   3 * (n_contact_points + num_surface_contacts))
						.applyOnTheLeft(R_local.transpose());
					G_data.G_inv
						.block(0,
							   3 * (n_contact_points + current_surface_contact),
							   3 * (n_contact_points + num_surface_contacts), 3)
						.applyOnTheRight(R_local);
				}

				current_surface_contact++;
			}
		}
	}
	return G_data;
}

GraspMatrixData SaiModel::environmentalGraspMatrix(
	const Vector3d& desired_resultant_point,
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	GraspMatrixData G_data = environmentalGraspMatrixAtGeometricCenter(
		resultant_in_world_frame, contact_forces_in_local_frames);

	Vector3d r_cg = G_data.resultant_point - desired_resultant_point;
	Matrix3d Rcross_cg = crossProductOperator(r_cg);

	int n = G_data.G.cols();

	G_data.G.block(3, 0, 3, n) += Rcross_cg * G_data.G.block(0, 0, 3, n);
	G_data.G_inv.block(0, 0, n, 3) -=
		G_data.G_inv.block(0, 3, n, 3) * Rcross_cg;

	G_data.resultant_point = desired_resultant_point;

	return G_data;
}

GraspMatrixData SaiModel::environmentalGraspMatrixAtGeometricCenter(
	const bool resultant_in_world_frame,
	const bool contact_forces_in_local_frames) const {
	vector<Vector3d> contact_locations;
	vector<ContactType> contact_types;
	int num_surface_contacts = 0;
	for (const auto& contact : _environmental_contacts) {
		if (resultant_in_world_frame) {
			contact_locations.push_back(
				positionInWorld(contact.contact_link_name, contact.contact_position));

		} else {
			contact_locations.push_back(
				position(contact.contact_link_name, contact.contact_position));
		}
		contact_types.push_back(contact.contact_type);
		if (contact.contact_type == SurfaceContact) {
			num_surface_contacts++;
		}
	}
	GraspMatrixData G_data =
		graspMatrixAtGeometricCenter(contact_locations, contact_types);

	if (contact_forces_in_local_frames) {
		int n_contact_points = _environmental_contacts.size();
		int current_surface_contact = 0;
		for (int i = 0; i < n_contact_points; i++) {
			Matrix3d R_local;
			if (resultant_in_world_frame) {
				R_local = rotationInWorld(
					_environmental_contacts[i].contact_link_name,
					_environmental_contacts[i].contact_orientation);
			} else {
				R_local =
					rotation(_environmental_contacts[i].contact_link_name,
							 _environmental_contacts[i].contact_orientation);
			}

			G_data.G
				.block(0, 3 * i, 3 * (n_contact_points + num_surface_contacts),
					   3)
				.applyOnTheRight(R_local);
			G_data.G_inv
				.block(3 * i, 0, 3,
					   3 * (n_contact_points + num_surface_contacts))
				.applyOnTheLeft(R_local.transpose());

			if (_environmental_contacts[i].contact_type == SurfaceContact) {
				G_data.G
					.block(0, 3 * (n_contact_points + current_surface_contact),
						   3 * (n_contact_points + num_surface_contacts), 3)
					.applyOnTheRight(R_local);
				G_data.G_inv
					.block(3 * (n_contact_points + current_surface_contact), 0,
						   3, 3 * (n_contact_points + num_surface_contacts))
					.applyOnTheLeft(R_local.transpose());

				if (n_contact_points > 2) {
					G_data.G
						.block(3 * (n_contact_points + current_surface_contact),
							   0, 3,
							   3 * (n_contact_points + num_surface_contacts))
						.applyOnTheLeft(R_local.transpose());
					G_data.G_inv
						.block(0,
							   3 * (n_contact_points + current_surface_contact),
							   3 * (n_contact_points + num_surface_contacts), 3)
						.applyOnTheRight(R_local);
				}

				current_surface_contact++;
			}
		}
	}
	return G_data;
}

void SaiModel::SaiModel::displayJoints() {
	cout << "\nRobot Joints :" << endl;
	for (map<string, int>::iterator it = _joint_names_to_id_map.begin();
		 it != _joint_names_to_id_map.end(); ++it) {
		cout << "joint : " << it->first << "\t id : " << it->second << endl;
	}
	cout << endl;
}

void SaiModel::displayLinks() {
	cout << "\nRobot Links :" << endl;
	for (map<string, int>::iterator it = _link_names_to_id_map.begin();
		 it != _link_names_to_id_map.end(); ++it) {
		cout << "link : " << it->first << "\t id : " << it->second << endl;
	}
	cout << endl;
}

void SaiModel::updateDynamics() {
	if (_M.rows() != _dof || _M.cols() != _dof) {
		_M.setZero(_dof, _dof);
	}

	CompositeRigidBodyAlgorithm(*_rbdl_model, _q, _M, false);
	updateInverseInertia();
}

void SaiModel::updateInverseInertia() { _M_inv = _M.inverse(); }

VectorXd SaiModel::modifiedNewtonEuler(const bool consider_gravity,
										const VectorXd& q, const VectorXd& dq,
										const VectorXd& dqa,
										const VectorXd& ddq) {
	VectorXd tau = VectorXd::Zero(_dof);

	vector<Vector3d> w, wa, dw, ddO, ddB, f, mom;
	vector<Vector3d> ripi_list, rib_list, z_list;
	Vector3d w_i, wa_i, dw_i, ddO_i, ddB_i, f_i, mom_i;
	Vector3d wp, wap, dwp, ddOp, ddBp, fc, tauc;
	Vector3d z, r_ipi, r_ipb, r_ib;

	// initial conditions forward recursion
	w_i.setZero();
	wa_i.setZero();
	dw_i.setZero();
	if (consider_gravity) {
		ddO_i = -_T_world_robot.linear().transpose() * _rbdl_model->gravity;
	} else {
		ddO_i.setZero();
	}
	ddB_i = ddO_i;

	f_i.setZero();
	mom_i.setZero();

	z.setZero();
	r_ipi.setZero();
	r_ib.setZero();

	w.push_back(w_i);
	wa.push_back(wa_i);
	dw.push_back(dw_i);
	ddO.push_back(ddO_i);
	ddB.push_back(ddB_i);

	z_list.push_back(z);
	rib_list.push_back(r_ib);
	ripi_list.push_back(r_ipi);

	f.push_back(f_i);
	mom.push_back(mom_i);

	for (int i = 1; i < _dof + 1; i++) {
		int parent = _rbdl_model->lambda_q[i];
		vector<unsigned int> children = _rbdl_model->mu[i];
		int child;
		if (children.empty()) {
			child = i;
			r_ipi.setZero();
		} else if (children.size() == 1) {
			child = _rbdl_model->mu[i][0];
			r_ipi = _rbdl_model->X_lambda[child].r;
		} else {
			throw("tree structures not implemented yet");
		}
		z = _rbdl_model->mJoints[i].mJointAxes->head(3);
		r_ipb = _rbdl_model->mBodies[i].mCenterOfMass;
		r_ib = -r_ipi + r_ipb;

		// transform parent quantities in local frame
		wp = _rbdl_model->X_lambda[i].E * w[parent];
		wap = _rbdl_model->X_lambda[i].E * wa[parent];
		dwp = _rbdl_model->X_lambda[i].E * dw[parent];
		ddOp = _rbdl_model->X_lambda[i].E * ddO[parent];

		w_i = wp + dq(parent) * z;
		wa_i = wap + dqa(parent) * z;
		dw_i = dwp + ddq(parent) * z + dq(parent) * wap.cross(z);
		ddO_i = ddOp + dw_i.cross(r_ipi) + w_i.cross(wa_i.cross(r_ipi));
		ddB_i = ddO_i + dw_i.cross(r_ib) + w_i.cross(wa_i.cross(r_ib));

		w.push_back(w_i);
		wa.push_back(wa_i);
		dw.push_back(dw_i);
		ddO.push_back(ddO_i);
		ddB.push_back(ddB_i);

		z_list.push_back(z);
		rib_list.push_back(r_ib);
		ripi_list.push_back(r_ipi);

		f.push_back(f_i);
		mom.push_back(mom_i);
	}

	// backward recursion
	for (int i = _dof; i > 0; i--) {
		Vector3d fip, mom_ip;
		vector<unsigned int> children = _rbdl_model->mu[i];
		if (children.size() == 0) {
			fip.setZero();
			mom_ip.setZero();
		} else if (children.size() == 1) {
			int child = children[0];
			fip = _rbdl_model->X_lambda[child].E.transpose() * f[child];
			mom_ip = _rbdl_model->X_lambda[child].E.transpose() * mom[child];
		} else {
			throw("tree structures not implemented yet");
		}

		double m = _rbdl_model->mBodies[i].mMass;
		Matrix3d I = _rbdl_model->mBodies[i].mInertia;

		f_i = fip + m * ddB[i];
		mom_i = mom_ip - f_i.cross(ripi_list[i] + rib_list[i]) +
				fip.cross(rib_list[i]) + I * dw[i] + wa[i].cross(I * w[i]);

		Vector3d zp = z_list[i];
		tau(i - 1) = mom_i.dot(zp);

		f[i] = f_i;
		mom[i] = mom_i;
	}
	return tau;
}

MatrixXd computePseudoInverse(const MatrixXd& matrix, const double& tolerance) {
	JacobiSVD<MatrixXd> svd(matrix, ComputeThinU | ComputeThinV);
	return svd.matrixV() *
		   (svd.singularValues().array().abs() > tolerance)
			   .select(svd.singularValues().array().inverse(), 0)
			   .matrix()
			   .asDiagonal() *
		   svd.matrixU().adjoint();
}

MatrixXd matrixRangeBasis(const MatrixXd& matrix, const double& tolerance) {
	const int range_size = matrix.rows();
	if (matrix.norm() < tolerance) {
		return MatrixXd::Zero(range_size, 1);
	}

	SelfAdjointEigenSolver<MatrixXd> es(matrix * matrix.transpose());

	double lambda_0 = es.eigenvalues()(range_size - 1);
	if (lambda_0 < tolerance) {
		return MatrixXd::Zero(range_size, 1);
	}

	const int max_range = min(matrix.rows(), matrix.cols());
	int task_dof = max_range;
	for (int i = (range_size - max_range); i < range_size; ++i) {
		if (es.eigenvalues()(i) / lambda_0 < tolerance) {
			task_dof -= 1;
		} else {
			break;
		}
	}

	if (task_dof == range_size) {
		return MatrixXd::Identity(range_size, range_size);
	} else {
		return es.eigenvectors().rightCols(task_dof).rowwise().reverse();
	}
}

Vector3d orientationError(const Matrix3d& desired_orientation,
						  const Matrix3d& current_orientation) {
	// check that the matrices are valid rotations
	Matrix3d Q1 = desired_orientation * desired_orientation.transpose() -
				  Matrix3d::Identity();
	Matrix3d Q2 = current_orientation * current_orientation.transpose() -
				  Matrix3d::Identity();
	if (Q1.norm() > 0.001 || Q2.norm() > 0.001) {
		cout << "des orientation:\n" << desired_orientation << endl;
		cout << "cur orientation:\n" << current_orientation << endl;
		cout << "Q1: " << Q1.norm() << endl;
		cout << "Q2: " << Q2.norm() << endl;
		throw invalid_argument(
			"Invalid rotation matrices in SaiModel::orientationError");
	} else {
		Vector3d rc1 = current_orientation.block<3, 1>(0, 0);
		Vector3d rc2 = current_orientation.block<3, 1>(0, 1);
		Vector3d rc3 = current_orientation.block<3, 1>(0, 2);
		Vector3d rd1 = desired_orientation.block<3, 1>(0, 0);
		Vector3d rd2 = desired_orientation.block<3, 1>(0, 1);
		Vector3d rd3 = desired_orientation.block<3, 1>(0, 2);
		return -1.0 / 2.0 * (rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
	}
}

Vector3d orientationError(const Quaterniond& desired_orientation,
						  const Quaterniond& current_orientation) {
	Quaterniond inv_dlambda =
		desired_orientation * current_orientation.conjugate();
	return -2.0 * inv_dlambda.vec();
}

Matrix3d crossProductOperator(const Vector3d& v) {
	Matrix3d v_hat;
	v_hat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
	return v_hat;
}

GraspMatrixData graspMatrixAtGeometricCenter(
	const vector<Vector3d>& contact_locations,
	const vector<ContactType>& contact_types) {
	MatrixXd G = MatrixXd::Zero(1, 1);
	MatrixXd G_inv = MatrixXd::Zero(1, 1);
	Matrix3d R = Matrix3d::Identity();
	Vector3d geometric_center = Vector3d::Zero();

	// number of contact points
	int n = contact_locations.size();
	if (n < 2) {
		throw invalid_argument(
			"invalid number of contact points (2 points min) in "
			"SaiModel::graspMatrixAtGeometricCenter\n");
	}
	if (n > 4) {
		throw invalid_argument(
			"invalid number of contact points (4 points max) in "
			"SaiModel::graspMatrixAtGeometricCenter\n");
	}
	if (contact_types.size() != n) {
		throw invalid_argument(
			"argument contact_locations and contact_types need to be of the "
			"same length in SaiModel::graspMatrixAtGeometricCenter\n");
	}

	// TODO : support line contact
	// number of surface contacts (that can apply a moment)
	int k = 0;
	for (int i = 0; i < n; i++) {
		if (contact_types[i] == SurfaceContact) {
			k++;
		}
	}

	// find geometric center
	for (int i = 0; i < n; i++) {
		geometric_center += contact_locations[i] / n;
	}

	// prepare Wf and Wm matrices
	MatrixXd Wf = MatrixXd::Zero(6, 3 * n);
	MatrixXd Wm = MatrixXd::Zero(6, 3 * k);

	for (int i = 0; i < n; i++) {
		Vector3d ri = contact_locations[i] - geometric_center;
		Wf.block<3, 3>(0, 3 * i) = Matrix3d::Identity();
		Wf.block<3, 3>(3, 3 * i) = crossProductOperator(ri);
	}
	for (int i = 0; i < k; i++) {
		Wm.block<3, 3>(3, 3 * i) = Matrix3d::Identity();
	}

	// prepare E and I
	MatrixXd E, I;

	Vector3d x, y, z;

	switch (n) {
		case 2: {
			// resize E
			E = MatrixXd::Zero(6, 1);

			// compute the point to point vectors
			Vector3d e12 = contact_locations[1] - contact_locations[0];
			double l = e12.norm();
			e12.normalize();

			// fill in E matrix
			E.block<3, 1>(0, 0) = -e12;
			E.block<3, 1>(3, 0) = e12;

			// create Ebar
			MatrixXd Ebar = 0.5 * E.transpose();

			// find R
			x = e12;
			if (abs((x.cross(Vector3d(1, 0, 0))).norm()) <
				1e-3)  // local x is aligned with world x
			{
				if (x.dot(Vector3d(1, 0, 0)) > 0)  // same direction
				{
					R = Matrix3d::Identity();
					// cout << "R is identity" << endl;
				} else	// rotation around Z axis by 180 degrees
				{
					R << -1, 0, 0, 0, -1, 0, 0, 0, 1;
				}
				x = R.col(0);
				y = R.col(1);
				z = R.col(2);
			} else {
				y = x.cross(Vector3d(1, 0, 0));
				y.normalize();
				z = x.cross(y);
				z.normalize();
				R.block<3, 1>(0, 0) = x;
				R.block<3, 1>(0, 1) = y;
				R.block<3, 1>(0, 2) = z;
			}

			switch (k) {
				case 0: {
					// throw runtime_error("Case 2-0 not implemented
					// yet\n");
					G.setZero(6, 6);
					G.block<3, 3>(0, 0) = Matrix3d::Identity();
					G.block<3, 3>(0, 3) = Matrix3d::Identity();
					G.block<1, 3>(3, 0) = l / 2.0 * z.transpose();
					G.block<1, 3>(3, 3) = -l / 2.0 * z.transpose();
					G.block<1, 3>(4, 0) = -l / 2.0 * y.transpose();
					G.block<1, 3>(4, 3) = l / 2.0 * y.transpose();
					G.block<1, 3>(5, 0) = -1.0 / 2.0 * x.transpose();
					G.block<1, 3>(5, 3) = 1.0 / 2.0 * x.transpose();

					G_inv.setZero(6, 6);
					G_inv.block<3, 3>(0, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 3>(3, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 1>(0, 3) = l * z;
					G_inv.block<3, 1>(3, 3) = -l * z;
					G_inv.block<3, 1>(0, 4) = -l * y;
					G_inv.block<3, 1>(3, 4) = l * y;
					G_inv.block<3, 1>(0, 5) = x;
					G_inv.block<3, 1>(3, 5) = x;

					break;
				}
				case 1: {
					// only 2 internal moments
					I = MatrixXd::Zero(2, 3);

					I << 0, 1, 0, 0, 0, 1;
					I = I * R.transpose();

					// populate G
					G = MatrixXd::Zero(9, 9);
					G.block<6, 6>(0, 0) = Wf;
					G.block<6, 3>(0, 6) = Wm;
					G.block<1, 6>(6, 0) = Ebar;
					G.block<2, 3>(7, 6) = I;

					// TODO : find explicit form for this case
					G_inv = MatrixXd::Zero(9, 9);
					G_inv = G.inverse();

					break;
				}
				case 2: {
					I = MatrixXd::Zero(5, 6);

					// find I
					I << -0.5, 0, 0, 0.5, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
						0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
					// I = I*Rr.transpose();
					I.block<5, 3>(0, 0) = I.block<5, 3>(0, 0) * R.transpose();
					I.block<5, 3>(0, 3) = I.block<5, 3>(0, 3) * R.transpose();

					// populate G
					G = MatrixXd::Zero(12, 12);
					G.block<6, 6>(0, 0) = Wf;
					G.block<6, 6>(0, 6) = Wm;
					G.block<1, 6>(6, 0) = Ebar;
					G.block<5, 6>(7, 6) = I;

					G_inv = MatrixXd::Zero(12, 12);
					Matrix3d rx_cross = crossProductOperator(x);
					Matrix3d rx_cross_square = rx_cross * rx_cross;
					G_inv.block<3, 3>(0, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 3>(3, 0) = 0.5 * Matrix3d::Identity();
					G_inv.block<3, 3>(0, 3) = rx_cross / l;
					G_inv.block<3, 3>(3, 3) = -rx_cross / l;
					G_inv.block<3, 3>(6, 3) =
						(Matrix3d::Identity() + rx_cross_square) / 2.0;
					G_inv.block<3, 3>(9, 3) =
						(Matrix3d::Identity() + rx_cross_square) / 2.0;

					G_inv.block<3, 1>(0, 6) = -x;
					G_inv.block<3, 1>(3, 6) = x;

					G_inv.block<3, 1>(6, 7) = -x;
					G_inv.block<3, 1>(9, 7) = x;

					G_inv.block<3, 1>(0, 8) = -z / l;
					G_inv.block<3, 1>(3, 8) = z / l;
					G_inv.block<3, 1>(6, 8) = y;

					G_inv.block<3, 1>(0, 9) = y / l;
					G_inv.block<3, 1>(3, 9) = -y / l;
					G_inv.block<3, 1>(6, 9) = z;

					G_inv.block<3, 1>(0, 10) = -z / l;
					G_inv.block<3, 1>(3, 10) = z / l;
					G_inv.block<3, 1>(9, 10) = y;

					G_inv.block<3, 1>(0, 11) = y / l;
					G_inv.block<3, 1>(3, 11) = -y / l;
					G_inv.block<3, 1>(9, 11) = z;

					break;
				}
				default:
					throw runtime_error(
						"Should not arrive here (number of contact points "
						"is "
						"2, number of surface contacts incoherent) in "
						"SaiModel::graspMatrixAtGeometricCenter\n");
			}
			break;
		}

		case 3: {
			// compute Wf_bar
			Matrix3d bot_right_Wf_WfT =
				Wf.block<3, 9>(3, 0) * Wf.block<3, 9>(3, 0).transpose();
			MatrixXd WfWfT_inv = MatrixXd::Identity(6, 6) / 3.0;
			WfWfT_inv.block<3, 3>(3, 3) = bot_right_Wf_WfT.inverse();

			MatrixXd Wf_bar = Wf.transpose() * WfWfT_inv;

			// resize E
			E = MatrixXd::Zero(9, 3);

			// compute the point to point vectors
			Vector3d e12 = contact_locations[1] - contact_locations[0];
			Vector3d e13 = contact_locations[2] - contact_locations[0];
			Vector3d e23 = contact_locations[2] - contact_locations[1];

			e12.normalize();
			e13.normalize();
			e23.normalize();

			// fill in E matrix
			E.block<3, 1>(0, 0) = -e12;
			E.block<3, 1>(3, 0) = e12;
			E.block<3, 1>(0, 1) = -e13;
			E.block<3, 1>(6, 1) = e13;
			E.block<3, 1>(3, 2) = -e23;
			E.block<3, 1>(6, 2) = e23;

			// create Ebar
			MatrixXd Ebar = (E.transpose() * E).inverse() * E.transpose();

			if (k < 0 || k > 3) {
				throw runtime_error(
					"Should not happen (number of contact points is 3, "
					"number "
					"of surface contacts incoherent) in "
					"SaiModel::graspMatrixAtGeometricCenter\n");
			} else if (k == 0) {
				// populate G
				G = MatrixXd::Zero(9, 9);
				G.block<6, 9>(0, 0) = Wf;
				G.block<3, 9>(6, 0) = Ebar;

				G_inv.setZero(9, 9);
				G_inv.block<9, 6>(0, 0) = Wf_bar;
				G_inv.block<9, 3>(0, 6) = E;
			} else {
				int tk = 3 * k;
				// compute I
				MatrixXd I = MatrixXd::Identity(tk, tk);

				// populate G
				G = MatrixXd::Zero(9 + tk, 9 + tk);
				G.block(0, 0, 6, 9) = Wf;
				G.block(0, 9, 6, tk) = Wm;
				G.block(6, 0, 3, 9) = Ebar;
				G.block(9, 9, tk, tk) = I;

				G_inv.setZero(9 + tk, 9 + tk);
				G_inv.block(0, 0, 9, 6) = Wf_bar;
				G_inv.block(0, 6, 9, 3) = E;
				G_inv.block(0, 9, 9, tk) = -Wf_bar * Wm;
				G_inv.block(9, 9, tk, tk) = I;
			}
			break;
		}

		case 4: {
			// compute Wf_bar
			Matrix3d bot_right_Wf_WfT =
				Wf.block<3, 12>(3, 0) * Wf.block<3, 12>(3, 0).transpose();
			MatrixXd WfWfT_inv = MatrixXd::Identity(6, 6) / 4.0;
			WfWfT_inv.block<3, 3>(3, 3) = bot_right_Wf_WfT.inverse();

			MatrixXd Wf_bar = Wf.transpose() * WfWfT_inv;

			// resize E
			E = MatrixXd::Zero(12, 6);

			// compute the point to point vectors
			Vector3d e12 = contact_locations[1] - contact_locations[0];
			Vector3d e13 = contact_locations[2] - contact_locations[0];
			Vector3d e14 = contact_locations[3] - contact_locations[0];
			Vector3d e23 = contact_locations[2] - contact_locations[1];
			Vector3d e24 = contact_locations[3] - contact_locations[1];
			Vector3d e34 = contact_locations[3] - contact_locations[2];

			e12.normalize();
			e13.normalize();
			e14.normalize();
			e23.normalize();
			e24.normalize();
			e34.normalize();

			// fill in E matrix
			E.block<3, 1>(0, 0) = -e12;
			E.block<3, 1>(3, 0) = e12;
			E.block<3, 1>(0, 1) = -e13;
			E.block<3, 1>(6, 1) = e13;
			E.block<3, 1>(0, 2) = -e14;
			E.block<3, 1>(9, 2) = e14;
			E.block<3, 1>(3, 3) = -e23;
			E.block<3, 1>(6, 3) = e23;
			E.block<3, 1>(3, 4) = -e24;
			E.block<3, 1>(9, 4) = e24;
			E.block<3, 1>(6, 5) = -e34;
			E.block<3, 1>(9, 5) = e34;

			// create Ebar
			MatrixXd Ebar = (E.transpose() * E).inverse() * E.transpose();

			if (k < 0 || k > 4) {
				throw runtime_error(
					"Should not arrive here (number of contact points is "
					"4, "
					"number of surface contacts incoherent) in "
					"SaiModel::graspMatrixAtGeometricCenter\n");
			} else if (k == 0) {
				// populate G
				G = MatrixXd::Zero(12, 12);
				G.block<6, 12>(0, 0) = Wf;
				G.block<6, 12>(6, 0) = Ebar;

				G_inv.setZero(12, 12);
				G_inv.block<12, 6>(0, 0) = Wf_bar;
				G_inv.block<12, 6>(0, 6) = E;
			} else {
				int tk = 3 * k;

				// compute I
				MatrixXd I = MatrixXd::Identity(tk, tk);

				// populate G
				G = MatrixXd::Zero(12 + tk, 12 + tk);
				G.block(0, 0, 6, 12) = Wf;
				G.block(0, 12, 6, tk) = Wm;
				G.block(6, 0, 6, 12) = Ebar;
				G.block(12, 12, tk, tk) = I;

				G_inv.setZero(12 + tk, 12 + tk);
				G_inv.block(0, 0, 12, 6) = Wf_bar;
				G_inv.block(0, 6, 12, 6) = E;
				G_inv.block(0, 12, 12, tk) = -Wf_bar * Wm;
				G_inv.block(12, 12, tk, tk) = I;
			}
			break;
		}

		default:
			throw runtime_error(
				"Should not arrive here (number of contact points is not "
				"2, 3 "
				"or 4) in SaiModel::graspMatrixAtGeometricCenter \n");
	}

	return GraspMatrixData(G, G_inv, R, geometric_center);
}

}  // namespace SaiModel
