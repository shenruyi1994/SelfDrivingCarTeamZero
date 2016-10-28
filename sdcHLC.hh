#ifndef __SDCHLC_HH__
#define __SDCHLC_HH__

namespace gazebo {
    class GAZEBO_VISIBLE sdcHLC {
    public:
        sdcHLC(sdcCar *car): car(car) {}
        ~sdcHLC();

        void update();
    private:
        sdcCar* car;
    };
}
#endif
