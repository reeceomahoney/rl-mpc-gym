//#include "a1.hpp"
#include "stance_controller.hpp"
#include <Eigen/Dense>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"


int main() {
    //Construct simulator
    raisim::World world;
    double time_step = 0.001;
    world.setTimeStep(time_step);
    raisim::RaisimServer server(&world);
    world.addGround();
    server.launchServer();

    //Create a1 class
    std::string urdf_path = "/home/romahoney/4yp/raisim_mpc/a1_data/urdf/a1.urdf";
    auto model = world.addArticulatedSystem(urdf_path);
    auto a1 = A1(model);
    world.integrate();

    VectorXd stance_duration {{0.3, 0.3, 0.3, 0.3}};
    VectorXd duty_factor {{0.6, 0.6, 0.6, 0.6}};
    VectorXi init_leg_state {{1, 1, 1, 1}};
    VectorXd init_leg_phase {{0.9, 0, 0, 0.9}};
    GaitGenerator gait_generator(stance_duration, duty_factor, init_leg_state, init_leg_phase);
    gait_generator.update(0);

    VectorXd desired_speed {{0, 0, 0}};
    StanceController stance_controller(a1, gait_generator, desired_speed, 0., 0.3, 12.0);
    
    std::vector<double> mpc_weights {1, 1, 0, 0, 0, 50, 0, 0, 1, 0.2, 0.2, 0.1, 0};
    auto action = stance_controller.getAction(mpc_weights);

    
    //Main loop
    for (int i=0; i<1e7; i++) {
        //TODO: changing sleep to step in real time
        world.integrate();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    server.killServer();
}