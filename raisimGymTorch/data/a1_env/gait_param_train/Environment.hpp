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
    Vector4d stance_duration_, duty_factor_, init_leg_phase_;
    Vector4i init_leg_state_;
    GaitProfile(string gait) {
      if (gait == "standing") {
        stance_duration_ << 0.3, 0.3, 0.3, 0.3;
        duty_factor_ << 1, 1, 1, 1;
        init_leg_phase_ << 0, 0, 0, 0;
        init_leg_state_ << 1, 1, 1, 1;
        }
      if (gait == "trotting") {
        stance_duration_ << 0.3, 0.3, 0.3, 0.3;
        duty_factor_ << 0.6, 0.6, 0.6, 0.6;
        init_leg_phase_ << 0.9, 0, 0, 0.9;
        init_leg_state_ << 0, 1, 1, 0;
        }
    }
};


VectorXd clip(VectorXd vec, double min, double max) {
  return vec.cwiseMax(min).cwiseMin(max);
}

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
    robot_ = std::make_unique<A1>(model_, simulation_dt_);
    
    // Setup controller
    setupController("trotting");
    controller_->reset();

    /// get model_ data
    gcDim_ = model_->getGeneralizedCoordinateDim();
    gvDim_ = model_->getDOF();
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);

    vel_.setZero(4);
    stance_duration_.setZero(4);
    duty_factor_.setZero(4);

    /// nominal configuration
    gc_init_ << 0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6;

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 37;
    actionDim_ = 21; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    // ACTION: mpc weights (12), mass (1), inertia tensor (6), stance duration (4), duty factor (4)

    /// action scaling
    actionMean_ << 1, 1, 0, 0, 0, 50, 0, 0, 1, 0.2, 0.2, 0.1, // mpc
      12.454, 0.07335, 0, 0, 0.25068, 0, 0.25447, // mass and inertia
      0.3, // stance duration
      0.6; // duty factor
    actionStd_ << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, // mpc
      0.3, 0.003, 0.003, 0.003, 0.03, 0.003, 0.03, // mass and inertia
      0.05, // stance duration
      0.05; // duty factor

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile(cfg["reward"]);

    /// visualize if it is the first environment
    if (visualizable_) {
      server_ = make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(model_);
    }
  }

  void init() final { }

  void reset() final {
    robot_->reset();
    controller_->reset();
    updateObservation();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    //Update the controller parameters. Set vel_rand to true for random commands
    generateCommand(robot_->getTimeSinceReset(), true);
    controller_->update(vel_(seq(0,2)), vel_(3));
    
    //Store for reward calculation
    robot_->desired_speed = vel_;

    /// action scaling
    action_ = action.cast<double>();
    action_ = action_.cwiseProduct(actionStd_);
    action_ += actionMean_;

    //Shift values to remove negatives
    auto min_weight = action_.minCoeff();
    if (min_weight < 0) {
      action_.array() -= min_weight;
    }
    
    // MPC weights
    std::vector<double> mpc_weights (action_.head(12).data(), 
      action_.head(12).data() + action_.head(12).rows() * action_.head(12).cols());
    mpc_weights.push_back(0.);

    // Dynamic parameters
    double mass = action_(12);
    VectorXd interita_tmp = action_(seq(13,18));
    std::vector<double> inertia (interita_tmp.data(), 
      interita_tmp.data() + interita_tmp.rows() * interita_tmp.cols());
    
    // Gait parameters
    stance_duration_.setConstant(action_(19));
    stance_duration_ = clip(stance_duration_, 0., 3.);
    duty_factor_.setConstant(action_(20));
    duty_factor_ = clip(duty_factor_, 0., 1.);

    // Send action, the first being an mpc step and the rest just regular steps
    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++) {
      mpc_step = false;
      if (i==0) { 
        mpc_step = true;
        controller_->setGaitParameters(stance_duration_, duty_factor_);
      }

      hybrid_action_ = controller_->getAction(mpc_step, mpc_weights, mass, inertia);
      hybrid_action_ = clip(hybrid_action_, -33.35, 33.25);   //Clip torques
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

    double torque_norm = hybrid_action_.squaredNorm();
    if (isnan(torque_norm)) torque_norm = 0;
    rewards_.record("torque", torque_norm);
    rewards_.record("error", robot_->getReward(bodyLinearVel_, bodyAngularVel_));
    rewards_.record("survival", 1);

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

    obDouble_ << vel_[0], vel_[1], vel_[3], // velocity commands
         gc_[2], // body height
        rot.e().row(2).transpose(), // body orientation
        gc_.tail(12), // joint angles
        bodyLinearVel_, bodyAngularVel_, // body linear & angular velocity
        gv_.tail(12); // joint velocity
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    // Check if robot has collapsed or jumped
    if ((robot_->getComPosition()[2] < 0.15) || (robot_->getComPosition()[2] > 0.45)) 
      return true;
    // Check if robot has rotated more than 45 degrees
    if ((abs(robot_->getBaseRollPitchYaw()[0]) > 0.79) || (abs(robot_->getBaseRollPitchYaw()[1]) > 0.79))
      return true;
    // Check if feet passes above body
    MatrixXd foot_pos = robot_->getFootPositionsInBaseFrame();
    for (int i=0; i<4; i++) {
      if (foot_pos(i, 2) > 0)
        return true;
    }

    terminalReward = 0.f;
    return false;
  }

  void curriculumUpdate() {};

  void setupController(string gait) {
    
    Vector3d vel_d {0, 0, 0};
    double desired_twisting_speed = 0;

    // Standing or trotting
    GaitProfile gait_profile(gait);

    gait_generator_ = make_unique<GaitGenerator>(gait_profile.stance_duration_, gait_profile.duty_factor_, 
        gait_profile.init_leg_state_, gait_profile.init_leg_phase_);

    sw_controller_ = make_unique<SwingController>(robot_.get(), gait_generator_.get(), vel_d, desired_twisting_speed, 
        robot_->mpc_body_height, 0.01);

    st_controller_ = make_unique<StanceController>(robot_.get(), gait_generator_.get(), vel_d, desired_twisting_speed,
        robot_->mpc_body_height, robot_->mpc_body_mass);

    controller_ = make_unique<LocomotionController>(robot_.get(), gait_generator_.get(), sw_controller_.get(), st_controller_.get());
  }

  void generateCommand(double t, bool vel_rand) {
  if (vel_rand == true) {
    if (std::fmod(t,1) == 0.) {
      vel_.setZero(4);

      //Randomly generate command
      int m = std::rand() % 6;
      switch (m) {
        case 0: vel_[0] = double(std::rand() % 225)/100; // vx (0 - 2.25)
          break;
        case 1: vel_[0] = -double(std::rand() % 225)/100;
          break;
        case 2: vel_[1] = double(std::rand() % 115)/100; // vy (0 - 1.15)
          break;
        case 3: vel_[1] = -double(std::rand() % 115)/100;
          break;
        case 4: vel_[3] = double(std::rand() % 300)/100; // wz (0 - 3)
          break;
        case 5: vel_[3] = -double(std::rand() % 300)/100;
          break;
      }
    }   
  }
  else {
    double vx = 0; double vy = 0; double wz = 0;

    VectorXd time_points {{0, 3, 6, 9, 12, 15, 18}};
    MatrixXd speed_points {{vx, 0, 0, 0}, {-vx, 0, 0, 0},{0, vy, 0, 0},
        {0, -vy, 0, 0}, {0, 0, 0, wz}, {0, 0, 0, -wz}};
    
    vel_ = interp1d(time_points, speed_points, t);
  }
}

 private:
  int gcDim_, gvDim_, nJoints_;
  bool mpc_step;

  unique_ptr<GaitGenerator> gait_generator_;
  unique_ptr<SwingController> sw_controller_;
  unique_ptr<StanceController> st_controller_;
  unique_ptr<LocomotionController> controller_;

  bool visualizable_ = false;
  raisim::ArticulatedSystem* model_;
  unique_ptr<A1> robot_;
  VectorXd gc_init_, gv_init_, gc_, gv_, action_, hybrid_action_;
  double terminalRewardCoeff_ = -10.;
  VectorXd obDouble_, actionMean_, actionStd_, vel_;
  VectorXd stance_duration_, duty_factor_;
  Vector3d bodyLinearVel_, bodyAngularVel_;
};
}