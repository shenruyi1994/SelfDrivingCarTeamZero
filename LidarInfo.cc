#include <stdio.h>
#include "LidarInfo.hh"

LidarInfo::LidarInfo() {
}

LidarInfo::LidarInfo(double minAngle, double resolution, double maxRange, int numRays) {
	this->minAngle = minAngle;
	this->resolution = resolution;
	this->maxRange = maxRange;
	this->numRays = numRays;
}
