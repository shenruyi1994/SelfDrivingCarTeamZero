#ifndef _dataProcessing_hh
#define _dataProcessing_hh


class sdcSensorData
{
	//camera data
	public:
		static void UpdateCameraData(int newLanePositionX, int newLanePositionY);
		static int[] LanePosition();

	private: 
		static int lanePositionX = 0;
		static int lanePositiony = 0;
