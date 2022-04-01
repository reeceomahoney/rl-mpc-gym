#include "a1.hpp"

#include <iostream>
#include <string>

#include "Eigen/Dense"
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

using namespace std;

//Change this to a relative path
string a1_urdf_path = "/home/romahoney/4yp/raisim_mpc/a1_data/urdf/a1.urdf";
Eigen::VectorXd init_pos {{0.0, 0.0, 0.3, 1.0, 0.0, 0.0, 0.0, 0.0, 0.8, -1.6, 
    0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 0.8, -1.6}};

Eigen::VectorXd sliceVecDyn(const raisim::VecDyn vec, int start_idx, int end_idx) {
    //Function to slice stuff like model.getGeneralizedCoordinate()
    Eigen::VectorXd vec2(end_idx - start_idx);
    int j = 0;

    for (int i = start_idx; i < end_idx; i++) {   
        vec2(j) = vec[i];
        j++;
    };
    
    return vec2;
};

class A1 {

    double time_step;
    //auto last_action; //TODO
    int num_legs = 4;
    int num_motors = 12;
    int step_counter = 0;

    //Motor gains
    Eigen::VectorXd motor_kp = 100 * Eigen::VectorXd::Ones(3);
    Eigen::VectorXd motor_kd {{1, 2, 2, 1, 2, 2, 1, 2, 2, 1, 2, 2}};

    //MPC parameters
    double mpc_body_mass = 12.454;
    Eigen::Matrix3d mpc_body_inertia {
        {0.07335, 0, 0},
        {0, 0.25068, 0},
        {0, 0, 0.25447}};
    double mpc_body_height = 0.30;

    raisim::ArticulatedSystem *model;
    
    A1(raisim::World world, double _time_step) {

        //Initialize model
        model = world.addArticulatedSystem(a1_urdf_path);
        model->setName("a1");
        model->setGeneralizedCoordinate(init_pos);
        model->getMassMatrix();
        world.integrate();

        time_step = _time_step;

    };

    void reset() {
    //TODO: Randomize initial states
    model->setGeneralizedCoordinate(init_pos);
    model->setGeneralizedVelocity(Eigen::VectorXd::Zero(18));
    step_counter = 0;
    //last_action = 0;
    };

    auto GetFootPositionsInBaseFrame() {};

    auto GetFootPosition() {
        model->getMassMatrix();
        auto com_position = model->getCompositeCOM()[0];
        return com_position;
    };

    auto GetComVelocity() {
        auto com_vel = sliceVecDyn(model->getGeneralizedVelocity(), 0, 3);
        //frametransform(com_vel)
    };

    auto frameTransformation(Eigen::VectorXd vec) {
        auto base_orientation = sliceVecDyn(
            model->getGeneralizedCoordinate(), 3, 7);
        auto w = base_orientation[0];
        //b_o[0:3] = [1:4] ...

    }



};