#pragma once

#include <stdlib.h>
#include <set>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "../../RaisimGymEnv.hpp"
#include "locomotion_controller.hpp"

//Helper function to interpolate velocity commands
VectorXd interp1d(VectorXd time_points, MatrixXd speed_points, double t) {
    for (int i = 0; i < time_points.size(); i++) {
        if ((t>=time_points[i]) && (t<=time_points[i+1])) {
            return speed_points.row(i);
        };
    };
};

class GaitProfile {
    public:
    Vector4d stance_duration, duty_factor, init_leg_phase;
    Vector4i init_leg_state;
    GaitProfile(string gait) {
        if (gait == "standing") {
            stance_duration << 0.3, 0.3, 0.3, 0.3;
            duty_factor << 1, 1, 1, 1;
            init_leg_phase << 0, 0, 0, 0;
            init_leg_state << 1, 1, 1, 1;
        };
        if (gait == "trotting") {
            stance_duration << 0.3, 0.3, 0.3, 0.3;
            duty_factor << 0.6, 0.6, 0.6, 0.6;
            init_leg_phase << 0.9, 0, 0, 0.9;
            init_leg_state << 0, 1, 1, 0;
        };
    };
};

LocomotionController setupController(A1* robot_, GaitGenerator* gg, SwingController* sw, 
  StanceController* st, string gait) {
    
    Vector3d desired_speed {0, 0, 0};
    double desired_twisting_speed = 0;

    // Standing or trotting
    GaitProfile gait_profile(gait);

    *gg = GaitGenerator(gait_profile.stance_duration, gait_profile.duty_factor, 
        gait_profile.init_leg_state, gait_profile.init_leg_phase);
    
    *sw = SwingController(robot_, gg, desired_speed, desired_twisting_speed, 
        robot_->mpc_body_height, 0.01);

    *st = StanceController(robot_, gg, desired_speed, desired_twisting_speed,
        robot_->mpc_body_height, robot_->mpc_body_mass);

    LocomotionController controller(robot_, gg, sw, st);

    return controller;
}

//Creates a speed profile
VectorXd getCommand_(double t, double vx, double vy, double wz) {
    VectorXd time_points {{0, 5}};
    MatrixXd speed_points {{vx, vy, 0, wz}};

    // Randomly generate mode
    int m = std::rand() % 3;
    switch (m) {
    case 0:
      speed_points(0,1) = 0;
      speed_points(0,3) = 0;
      break;
    case 1:
      speed_points(0,0) = 0;
      speed_points(0,3) = 0;
      break;
    case 2:
      speed_points(0,0) = 0;
      speed_points(0,1) = 0;
      break;
    }

    

    return interp1d(time_points, speed_points, t);
};

namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) {

    /// create world
    world_ = std::make_unique<raisim::World>();
    world_->addGround();

    /// add objects
    string a1_urdf_path = "/home/romahoney/4yp/raisim_mpc/a1_data/urdf/a1.urdf";
    model_ = world_->addArticulatedSystem(a1_urdf_path);
    model_->setName("a1");
    model_->setControlMode(raisim::ControlMode::FORCE_AND_TORQUE);
    *robot_ = A1(model_, simulation_dt_);
    
    *controller_ = setupController(robot_, gait_generator_, sw_controller_, st_controller_, "trotting");
    controller_->reset();

    /// get model_ data
    gcDim_ = model_->getGeneralizedCoordinateDim();
    gvDim_ = model_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);

    /// nominal configuration
    gc_init_ << 0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6;

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 34;
    actionDim_ = 13; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ << 1, 1, 0, 0, 0, 50, 0, 0, 1, 0.2, 0.2, 0.1, 0;
    actionStd_.setConstant(0.3);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile(cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.insert(9); footIndices_.insert(12); 
    footIndices_.insert(15); footIndices_.insert(18);

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(model_);
    }
  }

  void init() final { }

  void reset() final {
    robot_->reset();
    controller_->reset();
    vx = double((std::rand() % 35))/10 + 1.75; // 1.75 - 5.25
    vy = double((std::rand() % 17))/10 + 0.85; // 0.85 - 2.55
    wz = double((std::rand() % 40))/10 + 2; // 2 - 6
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    //Update the controller.arameters.
    auto desired_speed = getCommand_(robot_->getTimeSinceReset(), vx, vy, wz);
    controller_->update(desired_speed(seq(0,2)), desired_speed(3));
    
    //Store for reward calculation
    robot_->desired_speed = desired_speed;

    /// action scaling
    mpc_weights = action.cast<double>();
    mpc_weights = mpc_weights.cwiseProduct(actionStd_);
    mpc_weights += actionMean_;

    //Shift values to remove negatives
    auto min_weight = mpc_weights.minCoeff();
    if (min_weight < 0) {
      mpc_weights.array() -= min_weight;
    }

    std::vector<double> mpc_weights_(
        mpc_weights.data(), mpc_weights.data() + mpc_weights.rows() * mpc_weights.cols());
    
    // Send action, the first being an mpc step and the rest just regular steps
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      mpc_step = false;
      if (i==0) mpc_step = true;

      hybrid_action_ = controller_->getAction(mpc_step, mpc_weights_);
      robot_->step(hybrid_action_);
      
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();  
    }

    updateObservation();
    if (obDouble_.hasNaN()) {
      cout<<"Observation is nan, reseting state"<<endl;
      model_->setState(gc_init_, gv_init_);
      updateObservation();
    }

    rewards_.record("error", robot_->getReward(bodyLinearVel_, bodyAngularVel_));

    return rewards_.sum();
  }

  void updateObservation() {
    model_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);

    obDouble_ << gc_[2], /// body height
        rot.e().row(2).transpose(), /// body orientation
        gc_.tail(12), /// joint angles
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity
        gv_.tail(12); /// joint velocity
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    // Check if robot has collapsed
    if (robot_->getComPosition()[2] < 0.15) 
      return true;

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() {

  };

 private:
  int gcDim_, gvDim_, nJoints_;
  bool mpc_step;

  GaitGenerator* gait_generator_ = new GaitGenerator;
  SwingController* sw_controller_ = new SwingController;
  StanceController* st_controller_ = new StanceController;
  LocomotionController* controller_ = new LocomotionController;

  bool visualizable_ = false;
  raisim::ArticulatedSystem* model_;
  A1* robot_ = new A1;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, mpc_weights, hybrid_action_;
  double terminalRewardCoeff_ = -10., vx, vy, wz;
  Eigen::VectorXd obDouble_, actionMean_, actionStd_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  std::set<size_t> footIndices_;
};
}