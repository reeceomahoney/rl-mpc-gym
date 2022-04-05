#include "a1.hpp"
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

    //Main loop
    for (int i=0; i<1e7; i++) {
        //TODO: changing sleep to step in real time
        world.integrate();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    server.killServer();
}