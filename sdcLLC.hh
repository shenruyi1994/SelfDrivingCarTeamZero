#ifndef __SDCLLC_HH__
#define __SDCLLC_HH__

namespace gazebo {
    class GAZEBO_VISIBLE sdcLLC {
    public:
        sdcLLC(sdcCar *car): car(car) {}
        ~sdcLLC();

        void update();
    private:
        sdcCar* car;
    };
}
#endif
