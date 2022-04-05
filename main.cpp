#include <string>
#include <list>
#include <valarray>

#include "a1.hpp"
#include <Eigen/Dense>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

using namespace std;

int main() {
    //Construct simulator
    raisim::World world;
    double time_step = 0.001;
    world.setTimeStep(time_step);
    raisim::RaisimServer server(&world);
    world.addGround();
    server.launchServer();

    //Create a1 class
    string urdf_path = "/home/romahoney/4yp/raisim_mpc/a1_data/urdf/a1.urdf";
    auto model = world.addArticulatedSystem(urdf_path);
    auto a1 = A1(model);
    world.integrate();
    
    VectorXd force {{0.1, 0.5, -30}};
    auto x = a1.mapContactForceToJointTorques(0, force);
    
    for (auto &[k, v] : x) {
        cout << "x[" << k << "] = (" << v  << ") " << endl;
    };

    //Main loop
    for (int i=0; i<1e7; i++) {
        //TODO: changing sleep to step in real time
        world.integrate();
        this_thread::sleep_for(chrono::milliseconds(50));
    }
    
    server.killServer();
}