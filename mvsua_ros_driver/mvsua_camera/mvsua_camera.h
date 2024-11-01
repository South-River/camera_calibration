#ifndef MVSUA_CAMERA_H
#define MVSUA_CAMERA_H

#include "CameraApi.h"
#include <ros/ros.h>
// #include <opencv2/opencv.hpp>

#define MVSUA_CAMERA_MAX 5 //默认小于5个MVSUA相机

namespace camera_driver
{
    class mvsua_camera
    {
        public:
        mvsua_camera(int camera_num, int pixel_bit, std::string format);
        ~mvsua_camera();
        bool LoadParam(int camera_id,
                       int analog_gain, // 模拟增益
                       int gamma,       // gamma
                       int saturation, 
                       int shaprpness, 
                       int width, 
                       int height, 
                       int w_offset, 
                       int h_offset,
                       int speed,
                       int autoexposure_target,
                       double exposure_time);
        int iCameraCounts_;
        int pixel_bit_;
        std::string format_;
        private:
        bool MVSUA_Init();
        void MVSUA_UnInit();
        bool camera_initialized_;
        
        tSdkCameraCapbility g_tCapability_[MVSUA_CAMERA_MAX];
    };
}

#endif