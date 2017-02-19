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

}

void LidarPlugin::getVisibleObjects(std::vector<double>* objectRays) {
	sdcLidarRay left = sdcLidarRay(), right = sdcLidarRay();
	int leftIndex, rightIndex = -1;
	double minDistance = INT_MAX;
	bool objectIsDetected = false;
    sdcVisibleObject* object;
      
	for (int i = 0; i < 640; i++) {
		if (!isinf(objectRays->at(i)) && objectRays->at(i) < minDistance) {
			minDistance = objectRays->at(i);
		}
		if (!objectIsDetected && !isinf(objectRays->at(i))) {
			objectIsDetected = true;
			leftIndex = i;
		} else if (objectIsDetected && isinf(objectRays->at(i))) {
			rightIndex = i-1;

			sdcAngle leftAngle = sdcAngle((leftIndex-320)*lidarAngle);
			sdcAngle rightAngle = sdcAngle((rightIndex-320)*lidarAngle);

			sdcLidarRay left  = sdcLidarRay(leftAngle,objectRays->at(leftIndex));
			sdcLidarRay right = sdcLidarRay(rightAngle,objectRays->at(rightIndex));

		    object = new sdcVisibleObject(left, right, minDistance,leftIndex,rightIndex);
			break;
		}
	}
    
    //right side edge case
    if(0 <= leftIndex && leftIndex <= 639 && rightIndex == -1)
    {
        if (objectIsDetected) {
            sdcAngle leftAngle = sdcAngle((leftIndex-320)*lidarAngle);
            sdcAngle rightAngle = sdcAngle(320 * lidarAngle);

            sdcLidarRay left  = sdcLidarRay(leftAngle,objectRays->at(leftIndex));
            sdcLidarRay right = sdcLidarRay(rightAngle,objectRays->at(rightIndex));

            object = new sdcVisibleObject(left, right, minDistance, leftIndex, rightIndex);
        }
    }

	// checking if there are obstacles detected
	if (!objectIsDetected) {
		dataProcessing::UpdateIsNearbyObject(false);
        dataProcessing::UpdateObject(object);
	} else {
		dataProcessing::UpdateIsNearbyObject(true);
        sdcVisibleObject* oldObject = dataProcessing::GetNearbyObject();
        if (!object->IsSameObject(oldObject)) {
          dataProcessing::UpdateObject(object);
        }
	}


}
