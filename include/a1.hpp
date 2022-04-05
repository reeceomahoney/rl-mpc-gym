#include "raisim/World.hpp"

using namespace Eigen;

VectorXd sliceVecDyn(const raisim::VecDyn vec, int start_idx, int end_idx);

class A1 {

    double time_step;
    //auto last_action; //TODO
    int num_legs = 4;
    int num_motors = 12;
    int step_counter = 0;
    MatrixXd hip_offsets {
        {0.183, -0.048,  0.0},
        {0.183, 0.048, 0.0},
        {-0.183, -0.048, 0.0},
        {-0.183, 0.048, 0.0}};

    //Motor gains
    VectorXd motor_kp = 100 * VectorXd::Ones(3);
    VectorXd motor_kd {{1, 2, 2, 1, 2, 2, 1, 2, 2, 1, 2, 2}};

    //MPC parameters
    double mpc_body_mass = 12.454;
    Matrix3d mpc_body_inertia {
        {0.07335, 0, 0},
        {0, 0.25068, 0},
        {0, 0, 0.25447}};
    double mpc_body_height = 0.30;

    raisim::ArticulatedSystem* model;

    public:
    A1(raisim::ArticulatedSystem* _model);

    void reset();

    MatrixXd getFootPositionsInBaseFrame();
    VectorXd getComPosition();
    VectorXd getComVelocity();
    VectorXd frameTransformation(VectorXd vec);
    VectorXd getBaseRollPitchYaw();
    VectorXd getBaseRollPitchYawRate();

    std::tuple<Eigen::VectorXd, Eigen::VectorXd> 
    getJointAnglesFromLocalFootPosition(int leg_id, VectorXd foot_local_position);
    VectorXd getJointAngles();
    VectorXd getJointVelocities();
    VectorXd getObservation();

    double getTimeSinceReset();

    MatrixXd computeJacobian(int leg_id);
    std::map<int,double> mapContactForceToJointTorques(
        int leg_id, VectorXd contact_force);

    void step(VectorXd actions);
    double getReward();

};