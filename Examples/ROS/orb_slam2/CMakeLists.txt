cmake_minimum_required(VERSION 2.8.3)

project(orb_slam2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  cmake_modules)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

find_package(OpenCV  REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

catkin_package()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS
${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera
add_executable(mono src/ros_mono.cc)
target_link_libraries(mono ${LIBS})


# Node for RGB-D camera
add_executable(rgbd src/ros_rgbd.cc )
target_link_libraries(rgbd ${LIBS})

# Node for stereo camera
add_executable(stereo src/ros_stereo.cc)
target_link_libraries(stereo ${LIBS})

