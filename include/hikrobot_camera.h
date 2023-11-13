#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include <array>
#include "MV_CC_value_operation_factory.hpp"

namespace camera
{
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)
    //********** frame ************************************/
    extern cv::Mat frame;
    //********** frame_empty ******************************/
    extern bool frame_empty;
    //********** mutex ************************************/
    extern pthread_mutex_t mutex;

    //^ *********************************************************************************** //
    //^ ********************************** Camera Class************************************ //
    //^ *********************************************************************************** //
    class Camera
    {
    public:
        //********** 构造函数  ****************************/
        Camera(ros::NodeHandle &node);
        //********** 析构函数  ****************************/
        ~Camera();
        //********** 原始信息转换线程 **********************/
        static void *HKWorkThread(void *p_handle);

        //********** 输出摄像头信息 ***********************/
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        //********** 设置摄像头参数 ***********************/
        bool set(camera::CameraProperties type, float value);
        //********** 恢复默认参数 *************************/
        bool reset();
        //********** 读图1个相机的原始图像 ********************************/
        void ReadImg(cv::Mat &image);

        int get_width();
        int get_height();
        int get_Offset_x();
        int get_Offset_y();
        boost::array<double, 9> get_cameraMatrix();
        std::vector<double> get_distCoeffs();
        bool is_calibrate_enable();
        bool calibrate_enable();
        bool calibrate_disable();

    private:
        void undistort(cv::Mat &image, cv::Mat &image_rect);
        void image_resize(cv::Mat &src_image, cv::Mat &dst_image);
        //********** handle ******************************/
        void *handle_;
        //********** nThreadID ******************************/
        pthread_t nThreadID_;

        int     nRet_;

        //********** yaml config ******************************/
        int     acquisition_mode_;
        bool    frame_rate_enable_;
        int     frame_rate_;
        int     exposure_time_;
        bool    exposure_auto_;
        float   auto_exposure_time_lower_limit_;
        float   auto_exposure_time_upper_limit_;
        int     balance_white_auto_;
        int     brightness_;
        int     gain_auto_;
        bool    gamma_enable_;
        int     gamma_selector_;
        float   gamma_;
        int     height_;
        int     width_;
        int     Offset_x_;
        int     Offset_y_;
        int     pixel_format_;

        int     trigger_mode_;
        int     trigger_source_;
        int     line_selector_;

        MVCCValueOperationFactory* value_operator_ = NULL;

        bool    calibrate_enable_;
        cv::Mat cameraMatrix_;  // 相机内参
        cv::Mat distCoeffs_;     // 相机畸变校正参数

        bool    resize_enable_;
        int     dst_height_;
        int     dst_width_;

        int     compression_mode_;
        int     compression_quality_;
    };

} // namespace camera
#endif
