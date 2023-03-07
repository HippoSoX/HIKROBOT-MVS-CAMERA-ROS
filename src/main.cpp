#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.h"
#include "dynamic_reconfigure/server.h"
#include "hikrobot_camera/cameraConfig.h"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
// #define FIT_LIDAR_CUT_IMAGE false
// #if FIT_LIDAR_CUT_IMAGE
//     #define FIT_min_x 420
//     #define FIT_min_y 70
//     #define FIT_max_x 2450
//     #define FIT_max_y 2000
// #endif

boost::array<double, 12> K2P(boost::array<double, 9> K) {
    boost::array<double, 12> P = {0};
    P.at(0) = K.at(0);
    P.at(2) = K.at(2);
    P.at(5) = K.at(4);
    P.at(6) = K.at(5);
    P.at(10) = K.at(8);
    return P;
}

// void configCallback(hikrobot_camera::cameraConfig &config, uint32_t level, camera::Camera camera) {
//     if(config.CalibrateEnable == true) {
//         camera.calibrate_enable();
//     }
//     else {
//         camera.calibrate_disable();
//     }
// }

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src;
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    // 用image_transport::CameraPublisher发布的话题中compressed无法订阅
    // image_transport::CameraPublisher image_pub = main_cam_image.advertiseCamera("/hikrobot_camera/rgb", 1000);
    // 目前实测使用image_transport::Publisher发布的话题中compressed仍然无法订
    // *20220912 多次验证后，目前已可订阅compressed*
    image_transport::Publisher image_pub = main_cam_image.advertise("/hikrobot_camera/rgb", 1000);
    ros::Publisher camera_info_pub = hikrobot_camera.advertise<sensor_msgs::CameraInfo>("/hikrobot_camera/camera_info", 1000);

    // TODO Use threads to do dynamic reconfigure
    // dynamic_reconfigure::Server<hikrobot_camera::cameraConfig> server;
    // dynamic_reconfigure::Server<hikrobot_camera::cameraConfig>::CallbackType callbackFunc;

    // callbackFunc = boost::bind(&configCallback, _1, _2, MVS_cap);
    // server.setCallback(callbackFunc);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    
    camera_info_msg.height = MVS_cap.get_height();
    camera_info_msg.width = MVS_cap.get_width();
    camera_info_msg.distortion_model = "plumb_bob";
    if(MVS_cap.is_calibrate_enable()) {
        camera_info_msg.D = MVS_cap.get_distCoeffs();
        camera_info_msg.K = MVS_cap.get_cameraMatrix();
        camera_info_msg.P = K2P(MVS_cap.get_cameraMatrix());
    }
    camera_info_msg.roi.do_rectify = MVS_cap.is_calibrate_enable();
    camera_info_msg.roi.height = MVS_cap.get_height();
    camera_info_msg.roi.width = MVS_cap.get_width();
    camera_info_msg.roi.x_offset = MVS_cap.get_Offset_x();
    camera_info_msg.roi.y_offset = MVS_cap.get_Offset_y();
    
    ros::Rate loop_rate(100);

    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            continue;
        }
// #if FIT_LIDAR_CUT_IMAGE
//         cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
//         cv::Mat src_new = src(area);
//         cv_ptr->image = src_new;
// #else
        cv_ptr->image = src;
// #endif
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg.header.frame_id = "hikrobot_camera";

        camera_info_msg.header.frame_id = image_msg.header.frame_id;
	    camera_info_msg.header.stamp = image_msg.header.stamp;
        // image_pub.publish(image_msg, camera_info_msg);

        image_pub.publish(image_msg);
        camera_info_pub.publish(camera_info_msg);

        //*******************************************************************************************************************/
    }
    return 0;
}
