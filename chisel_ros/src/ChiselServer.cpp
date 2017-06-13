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

#include <chisel_ros/ChiselServer.h>
#include <chisel_ros/Conversions.h>
#include <chisel_ros/Serialization.h>
#include <visualization_msgs/Marker.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>

FILE *TSDF_time_file = fopen("/home/timer/chisel_update.txt","w");

namespace chisel_ros
{

ChiselServer::ChiselServer()
    : useColor(false), hasNewData(false), nearPlaneDist(0.05), farPlaneDist(5), isPaused(false), mode(FusionMode::DepthImage), calc_mesh(false)
{
}

ChiselServer::ChiselServer(const ros::NodeHandle &nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ,
                           float resolution, bool color, FusionMode fusionMode, bool _calc_mesh,
                           std::string camera_model_file, std::string mask_file)
    : nh(nodeHanlde), useColor(color), hasNewData(false), isPaused(false), mode(fusionMode), calc_mesh(_calc_mesh)
{
    chiselMap.reset(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
    cam.loadCameraFile(camera_model_file);
    cam.loadMask(mask_file);
    ROS_INFO("cameraModel loaded: width = %d, height = %d", cam.width, cam.height);
}

ChiselServer::~ChiselServer()
{
}

void ChiselServer::AdvertiseServices()
{
    resetServer = nh.advertiseService("Reset", &ChiselServer::Reset, this);
    pauseServer = nh.advertiseService("TogglePaused", &ChiselServer::TogglePaused, this);
    saveMeshServer = nh.advertiseService("SaveMesh", &ChiselServer::SaveMesh, this);
    getAllChunksServer = nh.advertiseService("GetAllChunks", &ChiselServer::GetAllChunks, this);
}

void ChiselServer::SetupMeshPublisher(const std::string &topic)
{
    meshTopic = topic;
    meshPublisher = nh.advertise<visualization_msgs::Marker>(meshTopic, 1);
}

void ChiselServer::SetupGridPublisher(const std::string &topic)
{
    gridTopic = topic;
    gridPublisher = nh.advertise<visualization_msgs::Marker>(gridTopic, 1);
}

void ChiselServer::PublishMeshes(const ros::Time &stamp)
{
    visualization_msgs::Marker marker;
    visualization_msgs::Marker marker2;
    FillMarkerTopicWithMeshes(&marker, &marker2, stamp);

    if (!marker2.points.empty())
    {
        if (calc_mesh) meshPublisher.publish(marker);
        gridPublisher.publish(marker2);
    }
}

void ChiselServer::SetupChunkBoxPublisher(const std::string &boxTopic)
{
    chunkBoxTopic = boxTopic;
    chunkBoxPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic, 1);
    latestChunkPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic + "/latest", 1);
}

visualization_msgs::Marker ChiselServer::CreateFrustumMarker(const chisel::Frustum &frustum)
{
    visualization_msgs::Marker marker;
    marker.id = 0;
    marker.header.frame_id = baseTransform;
    marker.color.r = 1.;
    marker.color.g = 1.;
    marker.color.b = 1.;
    marker.color.a = 1.;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

    marker.type = visualization_msgs::Marker::LINE_LIST;
    const chisel::Vec3 *lines = frustum.GetLines();
    for (int i = 0; i < 24; i++)
    {
        const chisel::Vec3 &linePoint = lines[i];
        geometry_msgs::Point pt;
        pt.x = linePoint.x();
        pt.y = linePoint.y();
        pt.z = linePoint.z();
        marker.points.push_back(pt);
    }

    return marker;
}

bool ChiselServer::TogglePaused(chisel_msgs::PauseService::Request &request, chisel_msgs::PauseService::Response &response)
{
    SetPaused(!IsPaused());
    return true;
}

bool ChiselServer::Reset(chisel_msgs::ResetService::Request &request, chisel_msgs::ResetService::Response &response)
{
    chiselMap->Reset();
    return true;
}

void ChiselServer::SubscribeAll(
                  const std::vector<std::string> &depth_imageTopic,
                  const std::vector<std::string> &color_imageTopic,
                  const std::string &transform, const std::vector<std::string> &odom_topic)
{
    depthCamera.resize(odom_topic.size());
    colorCamera.resize(odom_topic.size());
    sync.resize(odom_topic.size());
    lastDepthImage.resize(odom_topic.size());
    lastColorImage.resize(odom_topic.size());

    for (int i=0;i<odom_topic.size();i++)
    {
        printf("subscribe cam: %d\n",i);

        depthCamera[i].imageTopic = depth_imageTopic[i];
        depthCamera[i].transform = transform;
        depthCamera[i].sub_image = new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_imageTopic[i], 1);

        colorCamera[i].imageTopic = color_imageTopic[i];
        colorCamera[i].transform = transform;
        colorCamera[i].sub_image = new message_filters::Subscriber<sensor_msgs::Image>(nh, color_imageTopic[i], 1);

        colorCamera[i].sub_odom = new message_filters::Subscriber<nav_msgs::Odometry>(nh,odom_topic[i],1);

        sync[i] = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(1000),
                                                               *(depthCamera[i].sub_image),
                                                               *(colorCamera[i].sub_image),
                                                               *(colorCamera[i].sub_odom));

        if (i==0)
            sync[i]->registerCallback(boost::bind(&ChiselServer::CallbackAll_0, this, _1, _2, _3));
        else if (i==1)
            sync[i]->registerCallback(boost::bind(&ChiselServer::CallbackAll_1, this, _1, _2, _3));
        else
            sync[i]->registerCallback(boost::bind(&ChiselServer::CallbackAll_2, this, _1, _2, _3));
    }
}

void ChiselServer::CallbackAll_0(
    sensor_msgs::ImageConstPtr depth_image,
    sensor_msgs::ImageConstPtr color_image,
    nav_msgs::OdometryConstPtr odom)
{
    OdometryCallback(odom,0);

    ColorImageCallback(color_image,0);
    DepthImageCallback(depth_image,0);
}

void ChiselServer::CallbackAll_1(
    sensor_msgs::ImageConstPtr depth_image,
    sensor_msgs::ImageConstPtr color_image,
    nav_msgs::OdometryConstPtr odom)
{
ROS_WARN("Left: Before TSDF delay: %.0lfms",(ros::Time::now().toSec() - odom->header.stamp.toSec())*1000.0);
    OdometryCallback(odom,1);

    ColorImageCallback(color_image,1);
    DepthImageCallback(depth_image,1);
}

void ChiselServer::CallbackAll_2(
    sensor_msgs::ImageConstPtr depth_image,
    sensor_msgs::ImageConstPtr color_image,
    nav_msgs::OdometryConstPtr odom)
{
ROS_WARN("Right: Before TSDF delay: %.0lfms",(ros::Time::now().toSec() - odom->header.stamp.toSec())*1000.0);
    OdometryCallback(odom,2);

    ColorImageCallback(color_image,2);
    DepthImageCallback(depth_image,2);
}

void ChiselServer::OdometryCallback(nav_msgs::OdometryConstPtr odom, int i)
{
    chisel::Transform transform;
    transform.translation()(0) = odom->pose.pose.position.x;
    transform.translation()(1) = odom->pose.pose.position.y;
    transform.translation()(2) = odom->pose.pose.position.z;

    chisel::Quaternion quat;
    quat.x() = odom->pose.pose.orientation.x;
    quat.y() = odom->pose.pose.orientation.y;
    quat.z() = odom->pose.pose.orientation.z;
    quat.w() = odom->pose.pose.orientation.w;
    transform.linear() = quat.toRotationMatrix();

    colorCamera[i].lastPose = transform;
    depthCamera[i].lastPose = colorCamera[i].lastPose;

    colorCamera[i].gotPose = depthCamera[i].gotPose = true;
}

void ChiselServer::SetColorImage(const sensor_msgs::ImageConstPtr &img, int i)
{
    if (!lastColorImage[i].get())
    {
        lastColorImage[i].reset(ROSImgToColorImg<ColorData>(img));
    }

    ROSImgToColorImg(img, lastColorImage[i].get());

    colorCamera[i].lastImageTimestamp = img->header.stamp;
    colorCamera[i].gotImage = true;
}

void ChiselServer::SetDepthImage(const sensor_msgs::ImageConstPtr &img, int i)
{
    if (!lastDepthImage[i].get())
    {
        lastDepthImage[i].reset(new chisel::DepthImage<DepthData>(img->width, img->height));
    }

    ROSImgToDepthImg(img, lastDepthImage[i].get());
    depthCamera[i].lastImageTimestamp = img->header.stamp;
    depthCamera[i].gotImage = true;
}

void ChiselServer::DepthImageCallback(sensor_msgs::ImageConstPtr depthImage, int i)
{
    if (IsPaused())
        return;

ros::Time t1 = ros::Time::now();
ros::Time t3,t4,lock,unlock;

    SetDepthImage(depthImage, i);

    hasNewData = true;
    if (!IsPaused() && HasNewData())
    {
        //ROS_INFO("Got data.");
        auto start = std::chrono::system_clock::now();
        switch (GetMode())
        {
            case chisel_ros::ChiselServer::FusionMode::DepthImage:
lock = ros::Time::now();
                mtx.lock();
t3 = ros::Time::now();
                IntegrateLastDepthImage(i);
t4 = ros::Time::now();
                mtx.unlock();
unlock = ros::Time::now();
                break;
            case chisel_ros::ChiselServer::FusionMode::PointCloud:
                IntegrateLastPointCloud();
                break;
        }
        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start;
        ROS_INFO("CHISEL: Done with scan, %f ms", elapsed.count() * 1000);

        PublishChunkBoxes();
        if (chiselMap->GetMeshesToUpdate().size() == 0)
        {
            auto start = std::chrono::system_clock::now();
            //PublishMeshes(depthImage->header.stamp);
            std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start;
            //ROS_INFO("CHISEL: Done with publish, %f ms", elapsed.count() * 1000);
        }
       // else
            //PublishMeshes(depthImage->header.stamp);

ros::Time t2 = ros::Time::now();
//fprintf(TSDF_time_file,"%lf %d: %lf %lf %lf\n",(t2-t1).toSec()*1000.0,i,(t4-t3).toSec()*1000.0,(t3-lock).toSec()*1000.0,(unlock-t4).toSec()*1000.0);
//fflush(TSDF_time_file);

ROS_WARN("Cam %d: After TSDF delay: %.0lfms",i,(ros::Time::now().toSec() - depthImage->header.stamp.toSec())*1000.0);

        puts("");
    }
}

void ChiselServer::ColorImageCallback(sensor_msgs::ImageConstPtr colorImage, int i)
{
    if (IsPaused())
        return;
    SetColorImage(colorImage,i);
}

void ChiselServer::SubscribePointCloud(const std::string &topic)
{
    pointcloudTopic.cloudTopic = topic;
    pointcloudTopic.gotCloud = false;
    pointcloudTopic.gotPose = false;
    pointcloudTopic.cloudSubscriber = nh.subscribe(pointcloudTopic.cloudTopic, 20, &ChiselServer::PointCloudCallback, this);
}

void ChiselServer::PointCloudCallback(sensor_msgs::PointCloud2ConstPtr pointcloud)
{
    //ROS_INFO("PointCloudCallback");
    if (IsPaused())
        return;
    if (!lastPointCloud.get())
    {
        lastPointCloud.reset(new chisel::PointCloud());
    }
    ROSPointCloudToChisel(pointcloud, lastPointCloud.get());
    pointcloudTopic.transform = pointcloud->header.frame_id;

    pointcloudTopic.lastTimestamp = pointcloud->header.stamp;
    hasNewData = true;
}

void ChiselServer::SetupProjectionIntegrator(chisel::TruncatorPtr truncator, uint16_t weight, bool useCarving, float carvingDist)
{
    projectionIntegrator.SetCentroids(GetChiselMap()->GetChunkManager().GetCentroids());
    projectionIntegrator.SetTruncator(truncator);
    projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
    projectionIntegrator.SetCarvingDist(carvingDist);
    projectionIntegrator.SetCarvingEnabled(useCarving);
}

void ChiselServer::IntegrateLastDepthImage(int i)
{
    if (!IsPaused() && depthCamera[i].gotPose && lastDepthImage[i].get())
    {
ros::Time t1 = ros::Time::now();
        //ROS_ERROR("CHISEL: Integrating depth scan %d",i);
        auto start = std::chrono::system_clock::now();
        if (useColor)
        {
            chiselMap->IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator, lastDepthImage[i], depthCamera[i].lastPose, lastColorImage[i], cam);
        }
ros::Time t3 = ros::Time::now();
        std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - start;
        //ROS_INFO("CHISEL: Done with scan, %f ms", elapsed.count() * 1000);
        PublishLatestChunkBoxes();

        start = std::chrono::system_clock::now();
        //ROS_INFO("CHISEL: Updating meshes");
        chiselMap->UpdateMeshes();

        elapsed = std::chrono::system_clock::now() - start;
        //ROS_INFO("CHISEL: Done with mesh, %f ms", elapsed.count() * 1000);
        hasNewData = false;
ros::Time t2 = ros::Time::now();
fprintf(TSDF_time_file,"%lf\n",(t2-t1).toSec()*1000.0);
fflush(TSDF_time_file);

    }
}

void ChiselServer::IntegrateLastPointCloud()
{
    if (!IsPaused() && pointcloudTopic.gotPose && lastPointCloud.get())
    {
        //ROS_INFO("Integrating point cloud");
        chiselMap->IntegratePointCloud(projectionIntegrator, *lastPointCloud, pointcloudTopic.lastPose, 0.1f, farPlaneDist);
        PublishLatestChunkBoxes();
        chiselMap->UpdateMeshes();
        hasNewData = false;
    }
}

void ChiselServer::PublishLatestChunkBoxes()
{
    if (!latestChunkPublisher)
        return;
    const chisel::ChunkManager &chunkManager = chiselMap->GetChunkManager();
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = baseTransform;
    marker.ns = "chunk_box";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
    marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
    marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.3f;
    marker.color.g = 0.95f;
    marker.color.b = 0.3f;
    marker.color.a = 0.6f;
    const chisel::ChunkSet &latest = chiselMap->GetMeshesToUpdate();
    for (const std::pair<chisel::ChunkID, bool> &id : latest)
    {
        if (chunkManager.HasChunk(id.first))
        {
            chisel::AABB aabb = chunkManager.GetChunk(id.first)->ComputeBoundingBox();
            chisel::Vec3 center = aabb.GetCenter();
            geometry_msgs::Point pt;
            pt.x = center.x();
            pt.y = center.y();
            pt.z = center.z();
            marker.points.push_back(pt);
        }
    }

    latestChunkPublisher.publish(marker);
}

void ChiselServer::PublishChunkBoxes()
{
    const chisel::ChunkManager &chunkManager = chiselMap->GetChunkManager();
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = baseTransform;
    marker.ns = "chunk_box";
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
    marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
    marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = 0.95f;
    marker.color.g = 0.3f;
    marker.color.b = 0.3f;
    marker.color.a = 0.6f;
    for (const std::pair<chisel::ChunkID, chisel::ChunkPtr> &pair : chunkManager.GetChunks())
    {
        chisel::AABB aabb = pair.second->ComputeBoundingBox();
        chisel::Vec3 center = aabb.GetCenter();
        geometry_msgs::Point pt;
        pt.x = center.x();
        pt.y = center.y();
        pt.z = center.z();
        marker.points.push_back(pt);
    }

    chunkBoxPublisher.publish(marker);
}

chisel::Vec3 LAMBERT(const chisel::Vec3 &n, const chisel::Vec3 &light)
{
    return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
}

void ChiselServer::FillMarkerTopicWithMeshes(visualization_msgs::Marker *marker, visualization_msgs::Marker *marker2, const ros::Time &stamp)
{
    assert(marker != nullptr);
    assert(marker2 != nullptr);
    marker2->header.stamp = stamp;
    marker2->header.frame_id = baseTransform;
    marker2->ns = "grid";
    marker2->type = visualization_msgs::Marker::CUBE_LIST;
    marker2->scale.x = chiselMap->GetChunkManager().GetResolution();
    marker2->scale.y = chiselMap->GetChunkManager().GetResolution();
    marker2->scale.z = chiselMap->GetChunkManager().GetResolution();
    marker2->pose.orientation.x = 0;
    marker2->pose.orientation.y = 0;
    marker2->pose.orientation.z = 0;
    marker2->pose.orientation.w = 1;
    marker2->color.r = 1.0;
    marker2->color.g = 0.0;
    marker2->color.b = 0.0;
    marker2->color.a = 1.0;

    if (calc_mesh)
    {
        marker->header.stamp = stamp;
        marker->header.frame_id = baseTransform;
        marker->ns = "mesh";
        marker->scale.x = 1;
        marker->scale.y = 1;
        marker->scale.z = 1;
        marker->pose.orientation.x = 0;
        marker->pose.orientation.y = 0;
        marker->pose.orientation.z = 0;
        marker->pose.orientation.w = 1;
        marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
    }

    const chisel::MeshMap &meshMap = chiselMap->GetChunkManager().GetAllMeshes();

    if (meshMap.size() == 0)
    {
        //ROS_INFO("No Mesh");
        return;
    }

    chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
    lightDir.normalize();
    chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
    lightDir.normalize();
    const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);

    for (const std::pair<chisel::ChunkID, chisel::MeshPtr> &meshes : meshMap)
    {
        const chisel::MeshPtr &mesh = meshes.second;
        for (size_t i = 0; i < mesh->grids.size(); i++)
        {
            const chisel::Vec3 &vec = mesh->grids[i];
            geometry_msgs::Point pt;
            pt.x = vec[0];
            pt.y = vec[1];
            pt.z = vec[2];
            marker2->points.push_back(pt);
        }

        if (!calc_mesh) continue;

        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const chisel::Vec3 &vec = mesh->vertices[i];
            geometry_msgs::Point pt;
            pt.x = vec[0];
            pt.y = vec[1];
            pt.z = vec[2];
            marker->points.push_back(pt);

            if (mesh->HasColors())
            {
                const chisel::Vec3 &meshCol = mesh->colors[i];
                std_msgs::ColorRGBA color;
                color.r = meshCol[0];
                color.g = meshCol[1];
                color.b = meshCol[2];
                color.a = 1.0;
                marker->colors.push_back(color);
            }
            else
            {
                if (mesh->HasNormals())
                {
                    const chisel::Vec3 normal = mesh->normals[i];
                    std_msgs::ColorRGBA color;
                    chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                    color.r = fmin(lambert[0], 1.0);
                    color.g = fmin(lambert[1], 1.0);
                    color.b = fmin(lambert[2], 1.0);
                    color.a = 1.0;
                    marker->colors.push_back(color);
                }
                else
                {
                    std_msgs::ColorRGBA color;
                    color.r = vec[0] * 0.25 + 0.5;
                    color.g = vec[1] * 0.25 + 0.5;
                    color.b = vec[2] * 0.25 + 0.5;
                    color.a = 1.0;
                    marker->colors.push_back(color);
                }
            }
        }
    }
}

bool ChiselServer::SaveMesh(chisel_msgs::SaveMeshService::Request &request, chisel_msgs::SaveMeshService::Response &response)
{
    bool saveSuccess = chiselMap->SaveAllMeshesToPLY(request.file_name);
    return saveSuccess;
}

bool ChiselServer::GetAllChunks(chisel_msgs::GetAllChunksService::Request &request, chisel_msgs::GetAllChunksService::Response &response)
{
    const chisel::ChunkMap &chunkmap = chiselMap->GetChunkManager().GetChunks();
    response.chunks.chunks.resize(chunkmap.size());
    response.chunks.header.stamp = ros::Time::now();
    size_t i = 0;
    for (const std::pair<chisel::ChunkID, chisel::ChunkPtr> &chunkPair : chiselMap->GetChunkManager().GetChunks())
    {
        chisel_msgs::ChunkMessage &msg = response.chunks.chunks.at(i);
        FillChunkMessage(chunkPair.second, &msg);
        i++;
    }

    return true;
}

} // namespace chisel
