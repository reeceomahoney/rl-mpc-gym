#include "a1.hpp"

#include <iostream>
#include <string>
#include <math.h>
#include <tuple>

#include "Eigen/Dense"
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

using namespace std;
using namespace Eigen;

//Change this to a relative path
string a1_urdf_path = "/home/romahoney/4yp/raisim_mpc/a1_data/urdf/a1.urdf";
VectorXd init_pos {{0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0, 0.0, 0.8, -1.6, 
    0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6}};

VectorXd sliceVecDyn(const raisim::VecDyn vec, int start_idx, int end_idx) {
    //Function to slice stuff like model.getGeneralizedCoordinate()
    VectorXd vec2(end_idx - start_idx);
    int j = 0;

    for (int i = start_idx; i < end_idx; i++) {   
        vec2(j) = vec[i];
        j++;
    };
    
    return vec2;
};

VectorXd foot_position_in_hip_frame(VectorXd angles, int l_hip_sign) {
    double theta_ab = angles[0];
    double theta_hip = angles[1];
    double theta_knee = angles[2];

    double l_up = 0.2;
    double l_low = 0.2;
    double l_hip = 0.08505 * l_hip_sign;

    double leg_distance = sqrt(
        pow(l_up,2) + pow(l_low,2) + 2 * l_up * l_low * cos(theta_knee));
    double eff_swing = theta_hip + theta_knee / 2;

    double off_x_hip = -leg_distance * sin(eff_swing);
    double off_z_hip = -leg_distance * cos(eff_swing);
    double off_y_hip = l_hip;

    double off_x = off_x_hip;
    double off_y = cos(theta_ab) * off_y_hip - sin(theta_ab) * off_z_hip;
    double off_z = sin(theta_ab) * off_y_hip + cos(theta_ab) * off_z_hip;

    VectorXd foot_pos {{off_x, off_y, off_z}};
    return foot_pos;
};

VectorXd foot_position_in_hip_frame_to_joint_angle(VectorXd foot_position, int l_hip_sign) {
    double l_up = 0.2;
    double l_low = 0.2;
    double l_hip = 0.08505 * l_hip_sign;

    double x = foot_position[0];
    double y = foot_position[1];
    double z = foot_position[2];

    double theta_knee = -acos(
        (pow(x,2) + pow(y,2) + pow(z,2) - pow(l_hip,2) - pow(l_low,2) - pow(l_up,2)) /
        (2 * l_low * l_up));
    double l = sqrt(pow(l_up,2) + pow(l_low,2) + 2 * l_up * l_low * cos(theta_knee));
    double theta_hip = asin(-x / l) - theta_knee / 2;
    double c1 = l_hip * y - l * cos(theta_hip + theta_knee / 2) * z;
    double s1 = l * cos(theta_hip + theta_knee / 2) * y + l_hip * z;
    double theta_ab = atan2(s1, c1);

    VectorXd foot_pos {{theta_ab, theta_hip, theta_knee}};
    return foot_pos;
};

MatrixXd analytical_leg_jacobian(VectorXd leg_angles, int leg_id) {
  double l_up = 0.2;
  double l_low = 0.2;
  double l_hip = 0.08505 * pow(-1,leg_id + 1);

  double t1 = leg_angles[0];
  double t2 = leg_angles[1];
  double t3 = leg_angles[2];

  double l_eff = sqrt(pow(l_up,2) + pow(l_low,2) + 2 * l_up * l_low * cos(t3));
  double t_eff = t2 + t3 / 2;
  Matrix3d J;

  J(0,0) = 0;
  J(0,1) = -l_eff * cos(t_eff);
  J(0,2) = l_low * l_up * sin(t3) * sin(t_eff) / l_eff - l_eff * cos(
      t_eff) / 2;
  J(1,0) = -l_hip * sin(t1) + l_eff * cos(t1) * cos(t_eff);
  J(1,1) = -l_eff * sin(t1) * sin(t_eff);
  J(1,2) = -l_low * l_up * sin(t1) * sin(t3) * cos(
      t_eff) / l_eff - l_eff * sin(t1) * sin(t_eff) / 2;
  J(2,0) = l_hip * cos(t1) + l_eff * sin(t1) * cos(t_eff);
  J(2,1) = l_eff * sin(t_eff) * cos(t1);
  J(2,2) = l_low * l_up * sin(t3) * cos(t1) * cos(
      t_eff) / l_eff + l_eff * sin(t_eff) * cos(t1) / 2;

  return J;
};

//A1 CLASS METHODS//

A1::A1(raisim::ArticulatedSystem* _model) {

    //Initialize raisim model
    model = _model;
    model->setName("a1");
    model->setGeneralizedCoordinate(init_pos);
};

void A1::reset() {
    //TODO: Randomize initial states
    model->setGeneralizedCoordinate(init_pos);
    model->setGeneralizedVelocity(VectorXd::Zero(18));
    step_counter = 0;
    //last_action = 0;
};

MatrixXd A1::getFootPositionsInBaseFrame() {
    MatrixXd joint_angles = getJointAngles().reshaped<RowMajor>(4,3);
    MatrixXd foot_positions(4,3);

    for (int i = 0; i < 4; i++) {
        VectorXd tmp(joint_angles.row(i));
        foot_positions.row(i) = foot_position_in_hip_frame(
            tmp, pow(-1,i + 1));
    };

    return foot_positions + hip_offsets;
};

VectorXd A1::getComPosition() {
    model->getMassMatrix();
    auto tmp = model->getCompositeCOM()[0];

    Vector3d com;
    for (int i = 0; i < 3; i++) {
        com[i] = tmp[i];
    };
    
    return com;
};

VectorXd A1::getComVelocity() {
    auto com_vel = frameTransformation(
        sliceVecDyn(model->getGeneralizedVelocity(), 0, 3));
    return com_vel;
};

VectorXd A1::frameTransformation(VectorXd vec) {
    auto base_orientation = sliceVecDyn(
        model->getGeneralizedCoordinate(), 3, 7);
    
    //Convert to euler angles
    Quaterniond q;
    q.x() = base_orientation[0];
    q.y() = base_orientation[1];
    q.z() = base_orientation[2];
    q.w() = base_orientation[3];
    auto euler = q.toRotationMatrix().eulerAngles(0,1,2);

    //Create rotation matrix
    Quaterniond q2;
    q2 = AngleAxisd(0, Vector3d::UnitX())
    * AngleAxisd(0, Vector3d::UnitY())
    * AngleAxisd(-euler[2], Vector3d::UnitZ());
    auto rot_mat = q2.toRotationMatrix();
    
    VectorXd vec2 = rot_mat*vec;
    return vec2;
};

VectorXd A1::getBaseRollPitchYaw() {
    auto base_orientation = sliceVecDyn(
        model->getGeneralizedCoordinate(), 3, 7);
    
    //Convert to euler angles
    Quaterniond q;
    q.x() = base_orientation[0];
    q.y() = base_orientation[1];
    q.z() = base_orientation[2];
    q.w() = base_orientation[3];
    auto tmp = q.toRotationMatrix().eulerAngles(0,1,2);

    auto rpy = frameTransformation(tmp);
    return rpy;
};

VectorXd A1::getBaseRollPitchYawRate() {
    auto rpy_rate = frameTransformation(
        sliceVecDyn(model->getGeneralizedVelocity(), 3, 6));
    return rpy_rate;
};

std::tuple<Eigen::VectorXd, Eigen::VectorXd> 
    A1::getJointAnglesFromLocalFootPosition(int leg_id, 
    VectorXd foot_local_position) {

    VectorXd joint_position_idxs = VectorXd::LinSpaced(3, 3*leg_id, 3*(leg_id + 1));
    VectorXd joint_angles = foot_position_in_hip_frame_to_joint_angle(
        foot_local_position, pow(-1,leg_id + 1));

    return {joint_position_idxs, joint_angles};
};

VectorXd A1::getJointAngles() {
    auto joint_angles = sliceVecDyn(model->getGeneralizedCoordinate(), 7, 19);
    return joint_angles;
};

VectorXd A1::getJointVelocities() {
    auto joint_vels = sliceVecDyn(model->getGeneralizedVelocity(), 6, 18);
    return joint_vels;
};

VectorXd A1::getObservation() {
    auto com_position = getComPosition();
    auto com_velocity = getComVelocity();
    auto rpy = getBaseRollPitchYaw();
    auto rpy_rate = getBaseRollPitchYawRate();
    auto joint_angles = getJointAngles();
    auto joint_velocities = getJointVelocities();

    VectorXd obs(com_position.size() + com_velocity.size()+ rpy.size()
    + rpy_rate.size() + joint_angles.size() + joint_velocities.size());

    obs << com_position, com_velocity, rpy, rpy_rate,joint_angles, 
    joint_velocities;
    
    return obs;
};

double A1::getTimeSinceReset() {
    return step_counter * time_step;
};

MatrixXd A1::computeJacobian(int leg_id) {
    auto joint_angles = getJointAngles()(
        seq(3*leg_id, 3*(leg_id +1)));
    return analytical_leg_jacobian(joint_angles, leg_id);
};

std::map<int,double> A1::mapContactForceToJointTorques(
    int leg_id, VectorXd contact_force) {
        
        auto jv = computeJacobian(leg_id);
        VectorXd motor_torques_vec = jv.transpose()*contact_force;
        map<int, double> motor_torques_map;

        auto motor_ids = VectorXd::LinSpaced(3, 3*leg_id, 3*(leg_id+1));
        for (size_t i = 0; i < 3; i++) {
            motor_torques_map[motor_ids(i)] = motor_torques_vec[i];
        };
        
        return motor_torques_map;
};

void A1::step(VectorXd actions) {
    VectorXd forces = VectorXd::Zero(18);
    forces.tail(12) = actions;
    model->setGeneralizedForce(forces);
    step_counter++;
};

double A1::getReward() {
    auto com_velocity = getComVelocity();
    auto rpy_rate = getBaseRollPitchYaw();
    //TODO
};