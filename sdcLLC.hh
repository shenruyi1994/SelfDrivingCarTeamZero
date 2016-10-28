#include "sdcCar.hh"
#include "Waypoints.hh"

#ifndef __SDCLLC_HH__
#define __SDCLLC_HH__

namespace gazebo {
    class sdcCar;
    class GAZEBO_VISIBLE sdcLLC {
    public:
        sdcLLC(sdcCar *car): car(car) {}
        ~sdcLLC() {}

        void update();
        void setWaypoints(Waypoints waypoints);

    private:
        sdcCar* car;
        Waypoints waypoints;
    };
}
#endif
