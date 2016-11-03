#ifndef __SDCLLC_HH__
#define __SDCLLC_HH__

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "globals.hh"
#include "Waypoints.hh"

namespace gazebo {
    class sdcCar;
    class GAZEBO_VISIBLE sdcLLC {
    public:
        sdcLLC(sdcCar* car);
        ~sdcLLC() {}

        void update();
        void setWaypoints(Waypoints* waypoints);
        std::pair<double, double> calculateDubins(Waypoints* waypoints);

        // Control methods
        void Accelerate(double amt = 1, double rate = 1.0);
        void Brake(double amt = 1, double rate = 1.0);
        void Stop();
        void Reverse();
        void StopReverse();

    private:
        sdcCar* car_;
        Waypoints* waypoints_;
    };
}
#endif
