cmake_minimum_required(VERSION 3.0)

project(hikrobot_camera)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  camera_info_manager
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/camera.cfg
)

catkin_package()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  LINK_DIRECTORIES(/opt/MVS/lib/aarch64/) # ARM
  message(STATUS "Running on arrch64")
elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  LINK_DIRECTORIES(/opt/MVS/lib/64/)      # x64
  message(STATUS "Running on x86_64")
endif()

find_package(OpenCV REQUIRED)
message(STATUS "Find OpenCV version: "${OpenCV_VERSION})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
  /opt/MVS/include/
)

add_executable(${PROJECT_NAME}
  src/hikrobot_camera.cpp src/main.cpp)

add_dependencies(${PROJECT_NAME}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  #GCBase_gcc421_v3_0
  MvCameraControl
  #GenApi_gcc421_v3_0
  MVGigEVisionSDK
  #log4cpp_gcc421_v3_0
  MVRender
  #Log_gcc421_v3_0
  MvUsb3vTL
  #MathParser_gcc421_v3_0
  #NodeMapData_gcc421_v3_0
  MediaProcess
  #XmlParser_gcc421_v3_0
  X11
)