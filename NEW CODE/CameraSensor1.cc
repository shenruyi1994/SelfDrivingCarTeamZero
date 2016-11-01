//CameraSensor1.cc


#include "CameraSensor1.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraSensor1)

/////////////////////////////////////////////////
CameraSensor1::CameraSensor1() : SensorPlugin()
{
}

/////////////////////////////////////////////////
CameraSensor1::~CameraSensor1()
{
}

void CameraSensor1::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "CameraSensor1 requires a CameraSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&CameraSensor1::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);
}

void CameraSensor1::OnUpdate()
{
  // Pull raw data from camera sensor object as an unsigned character array with 3 channels.
  const unsigned char* img = this->parentSensor->GetImageData();
  int width = this->parentSensor->GetImageWidth()
  int height = this->parentSensor->GetImageHeight()
  Mat image = Mat(height, width,CV_8UC3, const_cast<unsigned char*>(img));

  // Adapted from chapter 7 of Computer Vision Application Programming Cookbook

  // use Canny algorithm to apply on the image to get contours. (args: input, output, low threshold, high threshold)
  cv::Mat contours;
  cv::Canny(img,contours,125,350);

  // invert the image to better show the lines (threshold value below 128 becomes 255)
  cv::Mat contoursInv;
  cv::threshold(contours,contoursInv,128, 255, cv::THRESH_BINARY_INV);

  // Hough tranform for line detection
  std::vector<cv::Vec2f> lines;
  cv::HoughLines(contoursInv,lines,168,1,PI/180,80);

  // leftmost edge and rightmost edge angles
  float leftEdgeAngle = PI;
  float rightEdgeAngle = PI;

  // Lines are then drawn by iterating over the lines vector
  std::vector<cv::Vec2f>::const_iterator it= lines.begin();
  while (it!=lines.end()) {
    float currentAngle = (*it)[1];

    if (0 < currentAngle <= (PI/2) && currentAngle < leftEdgeAngle) {
      leftEdgeAngle = currentAngle;
    } else if (PI/2 <= currentAngle < PI && currentAngle > rightEdgeAngle) {
      rightEdgeAngle = currentAngle;
    }
    ++it;
  }

  // Turning angle would be the average angels of left edge and right egde
	double turningAngle = (leftEdgeAngle+rightEdgeAngle)/2;

  // Need to make a decision with the steering amount

}
