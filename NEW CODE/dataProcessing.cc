#include "dataProcessing.hh"

//need to add namespace

//store the camera data
void dataProcessing::UpdateCameraData(int newLanePositionX, int newLanePositionY) {
    lanePositionX = newLanePositionX;
    lanePositionY = newLanePositionY;
}

int[] dataProcessing::LanePosition() {
	int[] lanePosition = int[2];
	lanePosition[0] = lanePositionX;
	lanePosition[1] = lanePositionY;
    return lanePosition;
}
