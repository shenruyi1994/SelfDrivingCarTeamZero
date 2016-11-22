cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


include_directories(FADBAD++)

find_package(CGAL REQUIRED COMPONENTS core)
include( ${CGAL_USE_FILE} )

if(NOT OpenCV_DIR)
 find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS    "/opt/ros/fuerte/share/OpenCV/")
endif(NOT OpenCV_DIR)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# link_directories(${OpenCV_LIBS}) Spits out error messages, and also doesn't seem necessary to work

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(sdcBox SHARED sdcBox.cpp)
target_link_libraries(sdcBox ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcFlyingBox SHARED sdcFlyingBox.cc)
target_link_libraries(sdcFlyingBox ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcAngle SHARED sdcAngle.cc)
target_link_libraries(sdcAngle ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcLidarSensorInfo SHARED sdcLidarSensorInfo.cc)
target_link_libraries(sdcLidarSensorInfo sdcAngle ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcLidarRay SHARED sdcLidarRay.cc)
target_link_libraries(sdcLidarRay sdcAngle ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcVisibleObject SHARED sdcVisibleObject.cc)
target_link_libraries(sdcVisibleObject sdcAngle sdcLidarRay ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcSensorData SHARED sdcSensorData.cc)

target_link_libraries(sdcSensorData sdcVisibleObject sdcLidarRay sdcLidarSensorInfo sdcAngle ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcFrontLidarSensor SHARED sdcFrontLidarSensor.cc)
target_link_libraries(sdcFrontLidarSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcTopLidarSensor SHARED sdcTopLidarSensor.cc)
target_link_libraries(sdcTopLidarSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcSideLidarSensor SHARED sdcSideLidarSensor.cc)
target_link_libraries(sdcSideLidarSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcBackLidarSensor SHARED sdcBackLidarSensor.cc)
target_link_libraries(sdcBackLidarSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcGpsSensor SHARED sdcGpsSensor.cc)
target_link_libraries(sdcGpsSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcCameraSensor SHARED sdcCameraSensor.cc)
target_link_libraries(sdcCameraSensor sdcSensorData ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(sdcWaypoint SHARED sdcWaypoint.cc)
target_link_libraries(sdcWaypoint ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(sdcIntersection SHARED sdcIntersection.cc)
target_link_libraries(sdcIntersection ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

find_package(CGAL REQUIRED COMPONENTS Core GMP RS3 MPFI )
include_directories(${CGAL_INCLUDE_DIRS})
set(LIBS ${LIBS} ${CGAL_LIBRARIES})

add_library(sdcBoundingBox SHARED sdcBoundingBox.cc)
target_link_libraries(sdcBoundingBox ${OpenCV_LIBS})

add_library(sdcRotatedBoundingBox SHARED sdcRotatedBoundingBox.cc)
target_link_libraries(sdcRotatedBoundingBox sdcBoundingBox ${OpenCV_LIBS})

add_library(sdcBoundingCircle SHARED sdcBoundingCircle.cc)
target_link_libraries(sdcBoundingCircle ${OpenCV_LIBS})

find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIRS})
set(LIBS ${LIBS} ${PROTOBUF_LIBRARIES})

add_library(LidarInfo SHARED LidarInfo.cc)
target_link_libraries(LidarInfo ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(data SHARED dataProcessing.cc)
target_link_libraries(data LidarInfo ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(lidar SHARED LidarPlugin.cc)
target_link_libraries(lidar data ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(camera SHARED CameraPlugin.cc)
target_link_libraries(camera data ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})

add_library(sdcSpline SHARED sdcSpline.cc)
target_link_libraries(sdcSpline sdcAngle ${OpenCV_LIBS})

add_library(sdcCar SHARED sdcCar.cc sdcHLC.cc sdcLLC.cc)
target_link_libraries(sdcCar data sdcWaypoint sdcIntersection sdcAngle sdcSensorData sdcBoundingBox sdcRotatedBoundingBox sdcBoundingCircle ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${CGAL_LIBRARIES} ${GOBJECT_LIBRARIES} ${OpenCV_LIBS})

