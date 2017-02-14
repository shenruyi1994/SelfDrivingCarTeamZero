#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "LidarPlugin.hh"

using namespace gazebo;

// register this plugin
GZ_REGISTER_SENSOR_PLUGIN(LidarPlugin)

double lidarAngle;

// lidar
LidarPlugin::LidarPlugin() : SensorPlugin()
{
}

LidarPlugin::~LidarPlugin()
{
}

void LidarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Get the parent sensor.
    this->parentSensor =
      boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a laser\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&LidarPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);

    lidarAngle = this->parentSensor->GetAngleResolution();

	dataProcessing::InitLidar(NEWFRONT, this->parentSensor->AngleMin().Radian(), this->parentSensor->GetAngleResolution(), this->parentSensor->GetRangeMax(), this->parentSensor->GetRayCount());
}

void LidarPlugin::OnUpdate()
{
	// vector that holds distance for each beam
	std::vector<double>* rays = new std::vector<double>();
	for (unsigned int i = 0; i < this->parentSensor->GetRayCount(); i++){
	  	rays->push_back(this->parentSensor->GetRange(i));
	}
	getVisibleObjects(rays);

	dataProcessing::UpdateLidarData(NEWFRONT, rays);

	// For each beam, print out the distance to an object.
	// If no object detect, prints out 'inf'
	/*rays = dataProcessing::GetLidarData(FRONT);
	std:: cout << "\nLidar Info\n";
	for(auto &i : *rays)
	{
		std:: cout << "(" << i << ") ";
	}*/
}

void LidarPlugin::getVisibleObjects(std::vector<double>* objectRays) {
	sdcLidarRay left = sdcLidarRay(), right = sdcLidarRay();
	int leftIndex, rightIndex = -1;
	double minDistance = INT_MAX;
	bool objectIsDetected = false;
	std::vector<sdcVisibleObject*> objectList;

	for (int i = 0; i < objectRays->size(); i++) {
		if (!isinf(objectRays->at(i)) && objectRays->at(i) < minDistance) {
			minDistance = objectRays->at(i);
		}
		if (!objectIsDetected && !isinf(objectRays->at(i))) {
			objectIsDetected = true;
			leftIndex = i;
		} else if (objectIsDetected && isinf(objectRays->at(i))) {
			objectIsDetected = false;
			rightIndex = i-1;

			sdcAngle leftAngle = sdcAngle((leftIndex-320)*lidarAngle);
			sdcAngle rightAngle = sdcAngle((rightIndex-320)*lidarAngle);

			sdcLidarRay left  = sdcLidarRay(leftAngle,objectRays->at(leftIndex));
			sdcLidarRay right = sdcLidarRay(rightAngle,objectRays->at(rightIndex));

			sdcVisibleObject* object = new sdcVisibleObject(left, right, minDistance,leftIndex,rightIndex);
			objectList.push_back(object);
			minDistance = INT_MAX;
		}

	}
	//right side edge case
	if (objectIsDetected) {
		sdcAngle leftAngle = sdcAngle((leftIndex-320)*lidarAngle);
		sdcAngle rightAngle = sdcAngle(lidarAngle*320);

		sdcLidarRay left  = sdcLidarRay(leftAngle,objectRays->at(leftIndex));
		sdcLidarRay right = sdcLidarRay(rightAngle,objectRays->at(rightIndex));

		sdcVisibleObject* object = new sdcVisibleObject(left, right, minDistance, leftIndex, rightIndex);
		objectList.push_back(object);
	}

	// checking if there are obstacles detected
	if (objectList.size() > 0) {
		dataProcessing::UpdateAreNearbyObjects(false);
	} else {
		dataProcessing::UpdateAreNearbyObjects(true);
	}
	dataProcessing::UpdateObjectList(objectList);
}
