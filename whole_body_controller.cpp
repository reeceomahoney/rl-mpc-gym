#include "locomotion_controller.hpp"
#include <Eigen/Dense>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

#include <time.h>


vector<double> mpc_weights {1, 1, 0, 0, 0, 50, 0, 0, 1, 0.2, 0.2, 0.1, 0};

double sim_freq = 1000;
int mpc_freq = 50;
int max_time = 40;

//Helper function to interpolate velocity commands
VectorXd interp1d(VectorXd time_points, MatrixXd speed_points, double t) {
    
    //Find which two values to interpolate between
    for (int i = 0; i < time_points.size(); i++) {
        if ((t>=time_points[i]) && (t<=time_points[i+1])) {
            double t1 = time_points[i];
            double t2 = time_points[i+1];
            VectorXd v1 = speed_points.row(i);
            VectorXd v2 = speed_points.row(i+1);
            
            return (v2 - v1)*(t - t1) / (t2 - t1) + v1;
        };
    };
};

//Creates a speed profile
VectorXd getCommand(double t) {
    double vx = 1.5;
    double vy = 0.75;
    double wz = 2.0;

    VectorXd time_points {{0, 5, 10, 15, 20, 25, 30, 35, 40}};
    MatrixXd speed_points {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, 
        {0, vy, 0, 0}, {0, -vy, 0, 0}, {0, 0, 0, wz}, {0, 0, 0, -wz}, 
        {0, 0, 0, 0}, {0, 0, 0, 0}};
    
    return interp1d(time_points, speed_points, t);
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

void updateController(
    LocomotionController* controller, VectorXd lin_speed, double ang_speed) {
        controller->swing_controller->desired_speed = lin_speed;
        controller->swing_controller->desired_twisting_speed = ang_speed;
        controller->stance_controller->desired_speed = lin_speed;
        controller->stance_controller->desired_twisting_speed = ang_speed;
};

int main() {
    //Construct simulator
    raisim::World world;
    double time_step = 1/sim_freq;
    world.setTimeStep(time_step);
    raisim::RaisimServer server(&world);
    world.addGround();
    server.launchServer();

    //Create a1 class
    std::string urdf_path = "/home/romahoney/4yp/raisim_mpc/a1_data/urdf/a1.urdf";
    auto model = world.addArticulatedSystem(urdf_path);
    A1 robot(model, time_step);
    A1* robot_ptr = &robot;
    
    //Create controller
    VectorXd desired_speed {{0, 0, 0}};
    double desired_twisting_speed = 0;

    //Modes: standing, trotting
    GaitProfile gait_profile("trotting");

    GaitGenerator gait_generator(
        gait_profile.stance_duration, 
        gait_profile.duty_factor, 
        gait_profile.init_leg_state, 
        gait_profile.init_leg_phase
    );
    GaitGenerator* gait_ptr = &gait_generator;

    SwingController sw_controller(
        robot_ptr, 
        gait_ptr,
        desired_speed,
        desired_twisting_speed, 
        robot_ptr->mpc_body_height, 
        0.01
    );
    SwingController* sw_ptr = &sw_controller;

    StanceController st_controller(
        robot_ptr,
        gait_ptr,
        desired_speed,
        desired_twisting_speed,
        robot_ptr->mpc_body_height,
        robot_ptr->mpc_body_mass
    );
    StanceController* st_ptr = &st_controller;

    LocomotionController controller(
        robot_ptr,
        gait_ptr,
        sw_ptr,
        st_ptr
    );
    LocomotionController* controller_ptr = &controller;

    controller.reset();
    
    auto start_time = robot.getTimeSinceReset();
    auto current_time = start_time;

    //Initialise so MPC always calls on the first step
    auto mpc_count = std::floor(sim_freq / mpc_freq) - 1;
    bool mpc_step;

    //Main loop
    while ((current_time - start_time) < max_time) {
        
        //Update the controller parameters.
        auto desired_speed = getCommand(current_time);
        updateController(controller_ptr, desired_speed(seq(0,2)), desired_speed(3));
        controller.update();

        //Store desired speed for reward function calcuation
        robot.desired_speed = desired_speed;

        //Flag to enforce MPC frequency
        mpc_count += 1;
        mpc_step = false;
        if (mpc_count == std::floor(sim_freq / mpc_freq)) {
            mpc_count = 0;
            mpc_step = true;
        };
        
        //Apply action
        auto hybrid_action = controller.getAction(mpc_step, mpc_weights);
        robot.step(hybrid_action);
        world.integrate();

        current_time = robot.getTimeSinceReset();
        std::this_thread::sleep_for(std::chrono::microseconds(50000));
        if (fmod(current_time,5.) == 0.) {    
            cout<<"Time: "<<current_time<<"s"<<endl;
        };
    };
    server.killServer();
};