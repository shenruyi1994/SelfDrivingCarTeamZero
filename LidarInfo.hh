#ifndef _LidarInfo_hh_
#define _LidarInfo_hh_

class LidarInfo {
	public:
		LidarInfo();
		LidarInfo(double minAngle, double resolution, double maxRange, int numRays);
		double minAngle;
		double resolution;
		double maxRange;
		int numRays;
};
#endif
