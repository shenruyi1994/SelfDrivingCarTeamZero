#include "sdcCar.hh"
#include "Waypoints.hh"

#ifndef __SDCHLC_HH__
#define __SDCHLC_HH__

namespace gazebo {
    class sdcCar;
    class GAZEBO_VISIBLE sdcHLC {
    public:
        sdcHLC(sdcCar *car): car(car) {}
        ~sdcHLC() {}

        void update();

    private:
        sdcCar* car;
        Waypoints* waypoints;
    };
}
#endif
