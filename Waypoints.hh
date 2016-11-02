#ifndef __WAYPOINT_HH__
#define __WAYPOINT_HH__

typedef struct {
    int x;
    int y;
} Waypoint;

namespace gazebo {
    class Waypoints {
    public:
        void addWaypoint();
        Waypoint* nextWaypoint();
    private:
        std::vector<Waypoint> waypoints_;
    };
}
#endif
