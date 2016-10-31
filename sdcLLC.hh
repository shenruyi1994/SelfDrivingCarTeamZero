#include "sdcCar.hh"
#include "Waypoints.hh"

#ifndef __SDCLLC_HH__
#define __SDCLLC_HH__

namespace gazebo {
    class sdcCar;
    class sdcHLC;
    class GAZEBO_VISIBLE sdcLLC {
    public:
        sdcLLC(sdcCar *car): car(car) {}
        ~sdcLLC() {}

        void update();
        void setWaypoints(Waypoints waypoints);

        // Control methods
        void Accelerate(double amt = 1, double rate = 1.0);
        void Brake(double amt = 1, double rate = 1.0);
        void Stop();
        void Reverse();
        void StopReverse();

    private:
        friend class sdcHLC;
        sdcCar* car;
        Waypoints waypoints;
    };
}
#endif
