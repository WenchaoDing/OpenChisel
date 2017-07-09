// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
    backward::SignalHandling sh;
} // namespace backward

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <chisel_ros/ChiselServer.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/truncation/InverseTruncator.h>
#include <boost/thread.hpp>

//#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_WARN

chisel_ros::ChiselServerPtr server;

void pub_map()
{
    ros::Rate loop_rate(10);

#ifdef PRINT_TIMING
	FILE *file = fopen("/home/timer/chisel_render.txt","w");
#endif

    while (ros::ok())
    {
        loop_rate.sleep();
ros::Time t1 = ros::Time::now();
        server->mtx.lock();
        server->PublishMeshes(ros::Time::now());
        server->mtx.unlock();
ros::Time t2 = ros::Time::now();

#ifdef PRINT_TIMING
fprintf(file,"%lf\n",(t2-t1).toSec()*1000.0);
fflush(file);
#endif
    }

#ifdef PRINT_TIMING
	fclose(file);
#endif
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting up chisel node.");
    ros::init(argc, argv, "Chisel");
    ros::NodeHandle nh("~");
    int chunkSizeX, chunkSizeY, chunkSizeZ;
    double voxelResolution;
    double truncationDistScale;
    int weight;
    bool useCarving;
    double carvingDist;
    std::string depthImageTopic;
    std::string depthImageTransform;
    std::string colorImageTopic;
    std::string colorImageTransform;
    std::string baseTransform;
    std::string meshTopic;
    std::string gridTopic;
    std::string chunkBoxTopic;
    double nearPlaneDist;
    double farPlaneDist;
    std::string pointCloudTopic;
    std::string odomTopic,transform_name;
    int number_of_threads;

    bool three_camera_mode;
    bool calc_mesh;
    std::string left_odom_topic, left_depth_image, left_color_image;
    std::string right_odom_topic, right_depth_image, right_color_image;

    std::vector<std::string> depthImageTopic_vec, colorImageTopic_vec, odomTopic_vec;
    std::string camera_model_file, mask_file;

    nh.param("chunk_size_x", chunkSizeX, 32);
    nh.param("chunk_size_y", chunkSizeY, 32);
    nh.param("chunk_size_z", chunkSizeZ, 32);
    nh.param("truncation_scale", truncationDistScale, 8.0);
    nh.param("integration_weight", weight, 1);
    nh.param("use_voxel_carving", useCarving, true);
    nh.param("carving_dist_m", carvingDist, 0.05);
    nh.param("voxel_resolution_m", voxelResolution, 0.03);
    nh.param("near_plane_dist", nearPlaneDist, 0.05);
    nh.param("far_plane_dist", farPlaneDist, 5.0);
    nh.param("base_transform", baseTransform, std::string("/camera_link"));
    nh.param("transform_name", transform_name, std::string("/camera_depth_optical_frame"));

    nh.param("three_camera_mode", three_camera_mode, false);
    nh.param("calc_mesh", calc_mesh, false);
    nh.param("number_of_threads", number_of_threads, 16);

    nh.param("camera_model_file", camera_model_file, std::string(""));
    nh.param("mask_file", mask_file, std::string(""));

    nh.param("odom_topic", odomTopic, std::string("/odom_topic"));
    nh.param("depth_image_topic", depthImageTopic, std::string("/depth_image"));
    nh.param("color_image_topic", colorImageTopic, std::string("/color_image"));

    nh.param("point_cloud_topic", pointCloudTopic, std::string("/camera/depth_registered/points"));

    // publish topic
    nh.param("mesh_topic", meshTopic, std::string("full_mesh"));
    nh.param("grid_topic", gridTopic, std::string("full_grid"));
    nh.param("chunk_box_topic", chunkBoxTopic, std::string("chunk_boxes"));
    
    depthImageTopic_vec.clear();
    colorImageTopic_vec.clear();
    odomTopic_vec.clear();
    depthImageTopic_vec.push_back(depthImageTopic);
    colorImageTopic_vec.push_back(colorImageTopic);
    odomTopic_vec.push_back(odomTopic);

    ROS_INFO("number_of_threads = %d", number_of_threads);

    if (three_camera_mode)
    {
        left_odom_topic = "/left_odom_topic";
        left_depth_image = "/left_depth_image";
        left_color_image = "/left_color_image";

        odomTopic_vec.push_back(left_odom_topic);
        depthImageTopic_vec.push_back(left_depth_image);
        colorImageTopic_vec.push_back(left_color_image);

        right_odom_topic = "/right_odom_topic";
        right_depth_image = "/right_depth_image";
        right_color_image = "/right_color_image";

        odomTopic_vec.push_back(right_odom_topic);
        depthImageTopic_vec.push_back(right_depth_image);
        colorImageTopic_vec.push_back(right_color_image);
    }

    ROS_INFO("Subscribing.");

    server = chisel_ros::ChiselServerPtr(new chisel_ros::ChiselServer(nh, chunkSizeX, chunkSizeY, chunkSizeZ,
                                         voxelResolution, calc_mesh, camera_model_file, mask_file, number_of_threads));

    //chisel::TruncatorPtr truncator(new chisel::QuadraticTruncator(truncationDistScale));
    chisel::TruncatorPtr truncator(new chisel::InverseTruncator(truncationDistScale));

    server->SetupProjectionIntegrator(truncator, static_cast<uint16_t>(weight), useCarving, carvingDist);

    server->SetNearPlaneDist(nearPlaneDist);
    server->SetFarPlaneDist(farPlaneDist);

    server->SubscribeAll(depthImageTopic_vec,
                         colorImageTopic_vec,
                         transform_name, odomTopic_vec);

    server->AdvertiseServices();

    server->SetBaseTransform(baseTransform);
    server->SetupMeshPublisher(meshTopic);
    server->SetupGridPublisher(gridTopic);
    server->SetupChunkBoxPublisher(chunkBoxTopic);
    ROS_INFO("Beginning to loop.");

    boost::thread pub_map_thread(pub_map);

    ros::spin();
    //ros::Rate loop_rate(100);

    //while (ros::ok())
    //{
    //    loop_rate.sleep();
    //    ros::spinOnce();
    //}
}