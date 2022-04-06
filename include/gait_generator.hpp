#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::VectorXi;

class GaitGenerator {
    public:
    VectorXd normalized_phase;
    VectorXi leg_state;     //0 = swing, 1 = stance
    VectorXi desired_leg_state;
    VectorXi next_leg_state;

    VectorXd stance_duration;
    VectorXd duty_factor;
    VectorXd swing_duration;
    VectorXd initial_leg_phase;
    VectorXi initial_leg_state;

    VectorXd initial_state_ratio_in_cycle;

    GaitGenerator();
    GaitGenerator(
        VectorXd _stance_duration, 
        VectorXd _duty_factor,
        VectorXi _initial_leg_state,
        VectorXd _initial_leg_phase
    );

    void reset();
    void update(double current_time);
};