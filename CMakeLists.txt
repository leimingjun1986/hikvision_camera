cmake_minimum_required(VERSION 3.0.2)
project(hikvision_camera)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  nodelet
  vision_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
 # INCLUDE_DIRS include
 #  LIBRARIES hikvision_camera
  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport nodelet roscpp sensor_msgs std_msgs vision_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
 add_library(${PROJECT_NAME}  
     src/hikvision_cameras.cpp
     src/vision_camera_nodelet.cpp
 )

 add_dependencies(${PROJECT_NAME}   ${catkin_EXPORTED_TARGETS}   )

 target_link_libraries(${PROJECT_NAME} 
            ${OpenCV_LIBS}
            ${catkin_LIBRARIES}
            "/opt/MVS/lib/64/libMvCameraControl.so"
            pthread.so
            libX11.so
            yaml-cpp)

  #add_dependencies(${PROJECT_NAME  ${PROJECT_NAME}_EXPORTED_TARGETS  ${catkin_EXPORTED_TARGETS}  )

  add_executable(hik_cam_test
              src/main_test.cpp 
  )

  target_link_libraries(hik_cam_test
          ${PROJECT_NAME} 
  )
 

