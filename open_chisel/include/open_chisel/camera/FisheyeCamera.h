#ifndef _FisheyeCamera_h
#define _FisheyeCamera_h

#include <memory>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/geometry/Frustum.h>
#include "camodocal/camera_models/CameraFactory.h"

#define UZH_OCAM

#ifdef UZH_OCAM
    #include <omni_cam/ocam.h>
#endif

namespace chisel
{
    class FisheyeCamera
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            FisheyeCamera();
            virtual ~FisheyeCamera();
            void setFarPlane(float _farPlane);
            void loadMask(std::string mask_file);
            void loadCameraFile(std::string camera_model_file);
            Vec3 ProjectPoint(const Vec3& point) const;
            Vec3 UnprojectPoint(const Vec3& point) const;
            void SetupFrustum(const Transform& view, Frustum* frustum) const;
            bool IsPointOnImage(const Vec3& point) const;

            int width, height;

        protected:

        #ifdef UZH_OCAM
            omni_cam::OCamPtr cam;
        #else
            camodocal::CameraPtr cam;
        #endif

            cv::Mat mask;            
            float farPlane;
    };
    typedef std::shared_ptr<FisheyeCamera> FisheyeCameraPtr;
    typedef std::shared_ptr<const FisheyeCamera> FisheyeCameraConstPtr;
} // namespace chisel 

#endif // _FisheyeCamera_h