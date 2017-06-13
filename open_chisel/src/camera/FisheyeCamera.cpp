#include <opencv2/opencv.hpp>
#include <open_chisel/camera/FisheyeCamera.h>

namespace chisel
{
    FisheyeCamera::FisheyeCamera()
    {
    }

    FisheyeCamera::~FisheyeCamera()
    {
    }

    void FisheyeCamera::setFarPlane(float _farPlane)
    {
        farPlane = _farPlane;
    }

    void FisheyeCamera::loadCameraFile(std::string camera_model_file)
    {
        farPlane = 0;        
    #ifdef UZH_OCAM
        cam = omni_cam::OCam::loadOCam(camera_model_file);
        width = cam->image_size_(0);
        height = cam->image_size_(1);
    #else
        cam = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(camera_model_file);
        width = cam->imageWidth();
        height = cam->imageHeight();
    #endif
        printf("FisheyeCamera init: width = %d, height = %d\n", width, height);
    }

    void FisheyeCamera::loadMask(std::string mask_file)
    {
        mask = cv::imread(mask_file.c_str(), 0);
    }

    // Projection (3D ---> 2D), depth in Euclidean distance
    Vec3 FisheyeCamera::ProjectPoint(const Vec3& point) const
    {
        const float& x = point(0);
        const float& y = point(1);
        const float& z = point(2);

        float norm = sqrt(x*x + y*y + z*z);

        Eigen::Vector3d P(x/norm, y/norm, z/norm);
        Eigen::Vector2d p_dst;
    #ifdef UZH_OCAM
        cam->project3(P, &p_dst, NULL);
    #else
        cam->spaceToPlane(P, p_dst);
    #endif

        return Vec3(p_dst(0), p_dst(1), norm);  // Vec3(u,v, Euclid distance)
    }

    // Back Projection (2D ---> 3D), depth in Euclidean distance
    Vec3 FisheyeCamera::UnprojectPoint(const Vec3& point) const
    {
        const float& u = point(0);
        const float& v = point(1);
        const float& z = point(2);

        Eigen::Vector2d p(u,v);
        Eigen::Vector3d P_dst;
    #ifdef UZH_OCAM
        cam->backProject3(p, &P_dst);
    #else
        cam->liftSphere(p, P_dst);
    #endif
        P_dst = P_dst * z;

        return Vec3(P_dst(0), P_dst(1), P_dst(2));
    }

    void FisheyeCamera::SetupFrustum(const Transform& view, Frustum* frustum) const
    {
        assert(frustum != nullptr);
        frustum->SetFromParams_Sphere(view, farPlane);
    }

    bool FisheyeCamera::IsPointOnImage(const Vec3& point) const
    {
        if (point(0) < 0 || point(1) < 0 || point(0) >= width || point(1) >= height)
            return false;
        uchar val = mask.at<uchar>(point(1), point(0));  // u,v
        if (val!=255)  // 255 - white - inside, 0 - black - outside
            return false;
        return true;
    }    
} // namespace chisel