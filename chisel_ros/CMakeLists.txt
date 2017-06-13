cmake_minimum_required(VERSION 2.8.3)
project(chisel_ros)

find_package(catkin REQUIRED COMPONENTS
          roscpp
          std_msgs
          sensor_msgs
          geometry_msgs
          tf
          open_chisel
          pcl_ros
          chisel_msgs
          camera_model
          omni_cam
        )

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${omni_cam_INCLUDE_DIRS}
    )

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++11 -O3")


catkin_package(CATKIN_DEPENDS roscpp tf std_msgs sensor_msgs open_chisel pcl_ros chisel_msgs
	 	INCLUDE_DIRS include
               	LIBRARIES ${PROJECT_NAME})


include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/ChiselServer.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} dw)
add_executable(ChiselNode src/ChiselNode.cpp)
target_link_libraries(ChiselNode ${PROJECT_NAME} ${catkin_LIBRARIES} dw)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".git" EXCLUDE
)
