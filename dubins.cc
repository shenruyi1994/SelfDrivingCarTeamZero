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
}

//True mod function that does not return negative values
double mod(double a, double b) {
  double ret = fmod(a,b);
  if (ret < 0)
    ret += b;
  return ret;
}

//Calculates a dubins path consisting of left turn, then straight .segment, then left turn
Path  lsl(double initD, double finalD, double dist) {
  //p is length of first turn, q is length of second turn, q is length of third turn
  double t, p, q;
  t = mod((-initD + atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD)))),2*PI);
  p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(initD)-sin(finalD)));
  q = mod(finalD - atan((cos(finalD)-cos(initD))/(dist+sin(initD)-sin(finalD))),2*PI);

  std::cout << "LSL-t is: " << t << ",  " << "LSL-p is: " <<  p << ",  " << "LSL-q is: " <<  q << "\n";

  PathDirection dirs [3] = { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT };
  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q, .dirs=dirs };
}


//Calculates a dubins path consisting of a right turn, then a straight .segment, then a right turn
Path rsr(double initD, double finalD, double dist) {
  double t,p,q;
  t = mod(initD - atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);
  p = sqrt(2 + dist*dist - 2*cos(initD - finalD) + 2*dist*(sin(finalD)-sin(initD)));
  q = mod(mod(-finalD,2*PI) + atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD))),2*PI);

  std::cout << "RSR-t is: " << t << ",  " << "RSR-p is: " <<  p << ",  " << "RSR-q is: " <<  q << "\n";

  PathDirection dirs [3] = { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT };
  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q, .dirs=dirs };
}

//Calculates a dubins path consisting of a right turn, then a straight .segment, then a left turn
Path rsl(double initD, double finalD, double dist) {
  double t,p,q;
  t = mod(initD - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);
  p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) - 2*dist*(sin(initD) + sin(finalD)));
  q = mod(mod(finalD, 2*PI) - atan((cos(initD)+cos(finalD))/(dist-sin(initD)-sin(finalD)))+atan(2/p),2*PI);

  std::cout << "RSL-t is: " << t << ",  " << "RSL-p is: " <<  p << ",  " << "RSL-q is: " <<  q << "\n";

  PathDirection dirs [3] = { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT };
  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q, .dirs=dirs };
}

//Calculates a dubins path consisting of a left turn, then a straight .segment, then a right turn
Path lsr(double initD, double finalD, double dist) {
  double t,p,q;
  t = mod(-initD + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);
  p = sqrt(dist*dist - 2 + 2*cos(initD-finalD) + 2*dist*(sin(initD) + sin(finalD)));
  q = mod(mod(-finalD, 2*PI) + atan((-cos(initD)-cos(finalD))/(dist+sin(initD)+sin(finalD)))-atan(-2/p),2*PI);

  std::cout << "LSR-t is: " << t << ",  " << "LSR-p is: " <<  p << ",  " << "LSR-q is: " <<  q << "\n";

  PathDirection dirs [3] = { DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT };
  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q, .dirs=dirs };
}

//Calculates a dubins path consisting of a right turn, then a left turn, then a right turn
Path rlr(double initD, double finalD, double dist) {
  double t,p,q;
  t = mod(initD-atan((cos(initD)-cos(finalD))/(dist-sin(initD)+sin(finalD)))+(p/2),2*PI);
  p = acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(sin(initD)-sin(finalD))));
  q = mod(initD-finalD-t+p,2*PI);

  std::cout << "RLR-t is: " << t << ",  " << "RLR-p is: " <<  p << ",  " << "RLR-q is: " <<  q << "\n";

  PathDirection dirs [3] = { DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT };
  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q, .dirs=dirs };
}

//Note:: there was a mistake in the paper discribing how to compute the lrl,
//I did my best to compute the solution, but this section needs to be tested further
//Calculates a dubins path consisting of a left turn, then a right turn, then a left turn
Path lrl(double initD, double finalD, double dist) {
  double t,p,q;
  p = mod(acos((1/8)*(6-dist*dist+2*cos(initD-finalD)+2*dist*(initD-finalD))),2*PI);
  t = mod(-initD-atan((cos(initD)-cos(finalD))/(dist+sin(initD)-sin(finalD)))+(p/2),2*PI);
  q = mod(mod(finalD,2*PI)-initD+2*p,2*PI);

  std::cout << "LRL-t is: " << t << ",  " << "LRL-p is: " <<  p << ",  " << "LRL-q is: " <<  q << "\n";

  PathDirection dirs [3] = { DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT };
  return (Path){ .seg1=t, .seg2=p, .seg3=q, .length=t+p+q, .dirs=dirs };
}

//Custom compare functor to compare lengths of dubins path
bool comparePath(Path &a, Path &b) {
  return a.length< b.length;
}

//Returns the minimum lenght dubins path
//Right now we are only checking the paths of form CSC, will implement CCC paths if needed
Path minPath(Path a, Path b, Path c, Path d) {
  std::vector<Path> paths;

  paths.push_back(a);
  paths.push_back(b);
  paths.push_back(c);
  paths.push_back(d);

  sort(paths.begin(), paths.end(), comparePath);

  return paths.front();
}



// Scales a path length by minimum turning radius, since we initially divided
// the distance to our waypoint by our minimum turning radius so we could
// calulate a dubins path with turning radius of 1 to make our calulations simpler
Path scalePath(Path dubinsPath) {
  dubinsPath.seg1 = dubinsPath.seg1*MIN_TURNING_RADIUS;
  dubinsPath.seg2 = dubinsPath.seg2*MIN_TURNING_RADIUS;
  dubinsPath.seg3 = dubinsPath.seg3*MIN_TURNING_RADIUS;
  dubinsPath.length = dubinsPath.length*MIN_TURNING_RADIUS;

  return dubinsPath;
}

//Converts a Path into a set of Controls, consisting of a turn direction and distance
std::vector<Control> dubins::pathToControls(Path dubinsPath) {
  double updateRate = 1000;
  double velocity = 6;
  Control control1, control2, control3;

  control1.direction = dubinsPath.dirs[0];
  control1.distance = dubinsPath.seg1;

  control2.direction = dubinsPath.dirs[1];
  control2.distance = dubinsPath.seg2;

  control3.direction = dubinsPath.dirs[2];
  control3.distance= dubinsPath.seg3;
  // std::cout << "First, turn " <<  direction1 << " for: " << control1.distance << " timesteps\n";
  //std::cout << "Then, turn " <<  direction2 << " for: " << control2.distance <<" timesteps\n";
  //std::cout << "Finally, turn " << direction3 <<" for: " << control3.distance <<" timesteps\n";
  return std::vector<Control> { control1, control2, control3 };
}



//Distance funciton between two 2d points
double distance2d(double x1, double x2, double y2, double y1 ){
  double dist= sqrt(pow(x2-x1,2)+pow(y2-y1,2));
  return dist;
}

//Main function to calculate a dubins path
//Calls functions to calculate each path individually, finds minimum lenght path assuming unit turning radius,  then scales path to proper length
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
  distance = distance/MIN_TURNING_RADIUS;

  //Calculate each type of dubins path individually
  Path lslP = lsl(initDirection, finalDirection, distance);
  Path lsrP = lsr(initDirection, finalDirection, distance);
  Path rsrP = rsr(initDirection, finalDirection, distance);
  Path rslP = rsl(initDirection, finalDirection, distance);
  // Path rlrP = rlr(initDirection, finalDirection, distance);
  // Path lrlP = lrl(initDirection, finalDirection, distance);

  //Returns path of smallest length
  Path dubinsPath = minPath(lslP,lsrP,rsrP,rslP);

  //Rescales path assuming unit turning radius to proper length
  dubinsPath = scalePath(dubinsPath);
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

