#include "sdcLLC.hh"
#include "sdcCar.hh"
#include "Waypoints.hh"
#include "sdcAngle.hh"
#include "dubins.hh"
#include "globals.hh"
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace gazebo;


dubins::dubins() {
  scalingFactor_ = 3.14159 * pow(MIN_TURNING_RADIUS, 2);
}

//True mod function that does not return negative values
double mod(double a, double b) {
  double ret = fmod(a,b);
  if (ret < 0)
    ret += b;
  return ret;
}

//Calculates a dubins path consisting of left turn, then straight .segment, then left turn
Path lsl(double initD, double finalD, double dist) {
  //p is length of first turn, q is length of second turn, q is length of third turn
  double t = mod((-initD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD)))),2*PI);
  double p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(initD)-sin(finalD)));
  double q = mod(finalD - atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD))),2*PI);

  std::cout << "LSL-t is: " << t << ",  " << "LSL-p is: " <<  p << ",  " << "LSL-q is: " <<  q << "\n";

  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q,
                 .dir1=DUBINS_LEFT, .dir2=DUBINS_STRAIGHT, .dir3=DUBINS_LEFT };
}


//Calculates a dubins path consisting of a right turn, then a straight .segment, then a right turn
Path rsr(double initD, double finalD, double dist) {
  double t = mod(initD - atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);
  double p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(finalD)-sin(initD)));
  double q = mod(mod(-finalD,2*PI) + atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);

  std::cout << "RSR-t is: " << t << ",  " << "RSR-p is: " <<  p << ",  " << "RSR-q is: " <<  q << "\n";

  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q,
                 .dir1=DUBINS_RIGHT, .dir2=DUBINS_STRAIGHT, .dir3=DUBINS_RIGHT };
}

//Calculates a dubins path consisting of a right turn, then a straight .segment, then a left turn
Path rsl(double initD, double finalD, double dist) {
  double p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) - 2*dist*(sin(initD) + sin(finalD)));
  double t = mod(initD - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);
  double q = mod(mod(finalD, 2*PI) - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);

  std::cout << "RSL-t is: " << t << ",  " << "RSL-p is: " <<  p << ",  " << "RSL-q is: " <<  q << "\n";

  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q,
                 .dir1=DUBINS_RIGHT, .dir2=DUBINS_STRAIGHT, .dir3=DUBINS_LEFT };
}

//Calculates a dubins path consisting of a left turn, then a straight .segment, then a right turn
Path lsr(double initD, double finalD, double dist) {
  double p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) + 2*dist*(sin(initD) + sin(finalD)));
  double t = mod(-initD + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);
  double q = mod(mod(-finalD, 2*PI) + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);

  std::cout << "LSR-t is: " << t << ",  " << "LSR-p is: " <<  p << ",  " << "LSR-q is: " <<  q << "\n";

  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q,
                 .dir1=DUBINS_LEFT, .dir2=DUBINS_STRAIGHT, .dir3=DUBINS_RIGHT };
}

//Calculates a dubins path consisting of a right turn, then a left turn, then a right turn
Path rlr(double initD, double finalD, double dist) {
  double p = acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(sin(initD)-sin(finalD))));
  double t = mod(initD-atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD)))+(p/2),2*PI);
  double q = mod(initD-finalD-t+p,2*PI);

  std::cout << "RLR-t is: " << t << ",  " << "RLR-p is: " <<  p << ",  " << "RLR-q is: " <<  q << "\n";

  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q,
                 .dir1=DUBINS_RIGHT, .dir2=DUBINS_LEFT, .dir3=DUBINS_RIGHT };
}

//Note:: there was a mistake in the paper discribing how to compute the lrl,
//I did my best to compute the solution, but this section needs to be tested further
//Calculates a dubins path consisting of a left turn, then a right turn, then a left turn
Path lrl(double initD, double finalD, double dist) {
  double p = mod(acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(initD-finalD))),2*PI);
  double t = mod(-initD-atan((cos(initD)-cos(finalD))/(dist+sin(initD)-sin(finalD)))+(p/2),2*PI);
  double q = mod(mod(finalD,2*PI)-initD+2*p,2*PI);

  std::cout << "LRL-t is: " << t << ",  " << "LRL-p is: " <<  p << ",  " << "LRL-q is: " <<  q << "\n";

  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q,
                 .dir1=DUBINS_LEFT, .dir2=DUBINS_RIGHT, .dir3=DUBINS_LEFT };
}

//Converts a Path into a set of Controls, consisting of a turn direction and distance
std::vector<Control> dubins::pathToControls(Path dubinsPath) {
  double updateRate = 1000;
  double velocity = 6;
  Control control1, control2, control3;

  control1.direction = dubinsPath.dir1;
  control1.distance = dubinsPath.seg1;

  control2.direction = dubinsPath.dir2;
  control2.distance = dubinsPath.seg2;

  control3.direction = dubinsPath.dir3;
  control3.distance= dubinsPath.seg3;
  // std::cout << "First, turn " <<  direction1 << " for: " << control1.distance << " timesteps\n";
  //std::cout << "Then, turn " <<  direction2 << " for: " << control2.distance <<" timesteps\n";
  //std::cout << "Finally, turn " << direction3 <<" for: " << control3.distance <<" timesteps\n";
  return std::vector<Control> { control1, control2, control3 };
}

// Distance funciton between two 2d points
double distance2d(double x1, double x2, double y2, double y1 ){
  double dist= sqrt(pow(x2-x1,2)+pow(y2-y1,2));
  return dist;
}

// Main function to calculate a dubins path
// Calls functions to calculate each path individually, finds minimum length
// path assuming unit turning radius,  then scales path to proper length
Path dubins::calculateDubins(std::vector<Waypoint> waypoints, Waypoint carpoint) {

  Waypoint testpoint = waypoints.front();


  double distance = distance2d(testpoint.x, carpoint.x, testpoint.y, carpoint.y);
  //double distance=sqrt(pow((testpoint.x-carpoint.x),2)+pow((testpoint.y-carpoint.y),2));
  double dubinsAngle = atan((testpoint.y-carpoint.y)/(testpoint.x-carpoint.x));
  std::cout << std::endl << "OUr dubins angle is" << dubinsAngle << std::endl;


  double initDirection=carpoint.direction;
  double finalDirection=testpoint.direction;
  //testpoints.push_back(testpoint);

  //Scale our distance, so we calculate dubins path length assuming a unit minimum turning radius
  distance = distance / scalingFactor_;

  //Calculate each type of dubins path individually
  std::vector<Path> paths;
  paths.push_back(lsl(initDirection, finalDirection, distance));
  paths.push_back(lsr(initDirection, finalDirection, distance));
  paths.push_back(rsr(initDirection, finalDirection, distance));
  paths.push_back(rsl(initDirection, finalDirection, distance));
  // paths.push_back(rlr(initDirection, finalDirection, distance));
  // paths.push_back(lrl(initDirection, finalDirection, distance));

  // Finds path of smallest length
  Path dubinsPath = paths[0];
  for (Path path : paths) {
    if (path.length < dubinsPath.length) {
      dubinsPath = path;
    }
  }

  //Rescales path assuming unit turning radius to proper length
  dubinsPath *= scalingFactor_;
  dubinsPath.origin.x = carpoint.x;
  dubinsPath.origin.y = carpoint.y;
  dubinsPath.origin.z = carpoint.direction;

  //Controls controls = pathToControls(dubinsPath);

  std::cout << "The minimum path "/* of type: " << dubinsPath.type */ << " is of length: " << dubinsPath.length << "\n";
  std::cout << "Seg 1 is length: " << dubinsPath.seg1 << "  .seg 2 is length: " << dubinsPath.seg2 << "  .seg 3 is length: " << dubinsPath.seg3 << "\n";

  // return controls;
  return dubinsPath;
}

//Left turn operator, given an initial waypoint, position, and distance to travel, return a point corresponding to a maximum left turn
cv::Point3d dubins::leftTurn(double x, double y, double theta, double dist) {
 cv::Point3d newPos;
 //dist = dist/MIN_TURNING_RADIUS;

 //theta= mod(theta, 2*PI);

 newPos.x = x+sin(theta + dist) - sin(theta);
 newPos.y = y-cos(theta + dist) + cos(theta);
 newPos.z = theta + dist;

  //std::cout << "Our new x coord is: " << newPos.x << "/n";
  //std::cout << "Our new y coord is: " << newPos.y << "/n";
  return newPos;
}

//Left turn operator, given an initial waypoint, position, and distance to travel, return a point corresponding to a maxim right turn
cv::Point3d dubins::rightTurn(double x, double y, double theta, double dist) {
  cv::Point3d newPos;

  //theta = mod(theta, 2*PI);

  //dist = dist/MIN_TURNING_RADIUS;

  newPos.x=x- sin(theta-dist)+sin(theta);
  newPos.y=y+cos(theta-dist)-cos(theta);
  newPos.z=theta-dist;

  //std::cout << "Our new x coord is: " << newPos.x << "/n";
  //std::cout << "Our new y coord is: " << newPos.y << "/n";

  return newPos;

}


//Left turn operator, given an initial waypoint, position, and distance to travel, return a point corresponding to a straight 'turn'
cv::Point3d dubins::straightTurn(double x, double y, double theta, double dist) {
  cv::Point3d newPos;

  //stheta= mod(theta, 2*PI);

  // dist = dist/MIN_TURNING_RADIUS;

  newPos.x = x+dist*cos(theta);
  newPos.y=y+dist*sin(theta);
  newPos.z = theta;

  //std::cout << "Our new x coord is: " << newPos.x << "/n";
  //std::cout << "Our new y coord is: " << newPos.y << "/n";

  return newPos;
}
