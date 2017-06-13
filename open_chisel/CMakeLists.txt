cmake_minimum_required(VERSION 2.8.3)
project(open_chisel)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++0x")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    camera_model
    omni_cam
)
catkin_package(INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME})

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
message("OpenCV_VERSION: ${OpenCV_VERSION}")
message("omni_cam_INCLUDE_DIRS: ${omni_cam_INCLUDE_DIRS}")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${omni_cam_INCLUDE_DIRS}
    )

add_library(${PROJECT_NAME}
    src/Chunk.cpp
    src/ChunkManager.cpp
    src/DistVoxel.cpp
    src/ColorVoxel.cpp
    src/geometry/AABB.cpp
    src/geometry/Plane.cpp
    src/geometry/Frustum.cpp
    src/camera/Intrinsics.cpp
    src/camera/PinholeCamera.cpp
    src/camera/FisheyeCamera.cpp
    src/pointcloud/PointCloud.cpp
    src/ProjectionIntegrator.cpp
    src/Chisel.cpp
    src/mesh/Mesh.cpp
    src/marching_cubes/MarchingCubes.cpp
    src/io/PLY.cpp
    src/geometry/Raycast.cpp)

target_link_libraries(${PROJECT_NAME} ${CATKIN_LIBRARIES} ${OpenCV_LIBS})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )
