#include <stdio.h>
#include <iostream>
#include "hikrobot_camera.h"
#include <chrono>
#include <cmath>

namespace camera {
    //********** frame ************************************/
    cv::Mat frame;
    //********** frame_empty ******************************/
    bool frame_empty = 0;
    //********** mutex ************************************/
    pthread_mutex_t mutex;
    
    int height;
    int width;

    Camera::Camera(ros::NodeHandle &node)
    {
        handle_ = NULL;
        cameraMatrix_ = cv::Mat::zeros(3, 3, CV_64F);
        distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
        node.param("width", width_, 1440);
        node.param("height", height_, 1080);
        camera::width = width_;
        camera::height = height_;
        node.param("FrameRateEnable", frame_rate_enable_, true);
        node.param("FrameRate", frame_rate_, 60);
        node.param("ExposureTime", exposure_time_, 2000);
        node.param("ExposureAuto", exposure_auto_, false);
        node.param("AutoExposureTimeLowerLimit", auto_exposure_time_lower_limit_, (float)100.);
        node.param("AutoExposureTimeUpperLimit", auto_exposure_time_upper_limit_, (float)16777.);
        node.param("BalanceWhiteAuot", balance_white_auto_, 2);
        node.param("Brightness", brightness_, 100);
        node.param("GammaEnable", gamma_enable_, false);
        node.param("GammaSelector", gamma_selector_, 1);
        node.param("Gamma", gamma_, (float)0.7);
        node.param("GainAuto", gain_auto_, 2);
        node.param("Offset_x", Offset_x_, 0);
        node.param("Offset_y", Offset_y_, 0);
        node.param("PixelFormat", pixel_format_, (int)BayerRG8);
        node.param("TriggerMode", trigger_mode_, 0);
        // node.param("TriggerSource", trigger_source_, 2);
        // node.param("LineSelector", line_selector_, 2);
        node.param("ResizeEnable", resize_enable_, false);
        if(resize_enable_) {
            node.param("DestinationHeight", dst_height_, 640);
            node.param("DestinationWidth", dst_width_, 640);
        }
        node.param("ImageCompressionMode", compression_mode_, 1);
        node.param("ImageCompressionQuality", compression_quality_, 50);

        node.param("CalibrateEnable", calibrate_enable_, false);

        if(calibrate_enable_) {
            double fx, fy, cx, cy, k1, k2, p1, p2, k3;

            node.param("CameraMatrix/fx", fx, 1.);
            node.param("CameraMatrix/fy", fy, 1.);
            node.param("CameraMatrix/cx", cx, 0.);
            node.param("CameraMatrix/cy", cy, 0.);
            cameraMatrix_.at<double>(0,0) = fx;
            cameraMatrix_.at<double>(1,1) = fy;
            cameraMatrix_.at<double>(0,2) = cx;
            cameraMatrix_.at<double>(1,2) = cy;
            cameraMatrix_.at<double>(2,2) = 1.;

            node.param("DistCoeffs/k1", k1, 0.);
            node.param("DistCoeffs/k2", k2, 0.);
            node.param("DistCoeffs/p1", p1, 0.);
            node.param("DistCoeffs/p2", p2, 0.);
            node.param("DistCoeffs/k3", k3, 0.);
            distCoeffs_.at<double>(0,0) = k1;
            distCoeffs_.at<double>(1,0) = k2;
            distCoeffs_.at<double>(2,0) = p1;
            distCoeffs_.at<double>(3,0) = p2;
            distCoeffs_.at<double>(4,0) = k3;
        }

        uint32_t sdk_v = MV_CC_GetSDKVersion();
        printf("SDK Version: v%d.%d.%d.%d\n", (sdk_v&0xFF000000)>>24, (sdk_v&0x00FF0000)>>16, (sdk_v&0x0000FF00)>>8, sdk_v&0x000000FF);

        //********** 枚举设备 ********************************/
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet_ = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet_)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet_);
            exit(-1);
        }
        unsigned int nIndex = 0;
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        //********** 选择设备并创建句柄 *************************/

        nRet_ = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[0]);

        if (MV_OK != nRet_)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet_);
            exit(-1);
        }

        // 打开设备
        //********** frame **********/

        nRet_ = MV_CC_OpenDevice(handle_);

        if (MV_OK != nRet_)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet_);
            exit(-1);
        }

        //设置 yaml 文件里面的配置
        value_operator_ = new MVCCValueOperationFactory(&handle_);
        this->reset();
        if(MV_OK != nRet_) {
            printf("set parameter error");
        }

        //软件触发
        // ********** frame **********/
        // nRet_ = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
        // if (MV_OK == nRet_)
        // {
        //     printf("set TriggerMode OK!\n");
        // }
        // else
        // {
        //     printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet_);
        // }
        MVCC_ENUMVALUE t = {0};
        //********** frame **********/

        nRet_ = MV_CC_GetEnumValue(handle_, "PixelFormat", &t);

        if (MV_OK == nRet_)
        {
            printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet_);
        }
        // 开始取流
        //********** frame **********/

        nRet_ = MV_CC_StartGrabbing(handle_);

        if (MV_OK != nRet_)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet_);
            exit(-1);
        }
        //初始化互斥量
        nRet_ = pthread_mutex_init(&mutex, NULL);
        if (nRet_ != 0)
        {
            perror("pthread_create failed\n");
            exit(-1);
        }
        //********** frame **********/

        nRet_ = pthread_create(&nThreadID_, NULL, HKWorkThread, handle_);

        if (nRet_ != 0)
        {
            printf("thread create failed.ret = %d\n", nRet_);
            exit(-1);
        }
    }

    Camera::~Camera()
    {
        int nRet;
        //********** frame **********/

        pthread_join(nThreadID_, NULL);

        delete value_operator_;

        //********** frame **********/

        nRet = MV_CC_StopGrabbing(handle_);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        // 关闭设备
        //********** frame **********/

        nRet = MV_CC_CloseDevice(handle_);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        // 销毁句柄
        //********** frame **********/

        nRet = MV_CC_DestroyHandle(handle_);

        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");
        // 销毁互斥量
        pthread_mutex_destroy(&mutex);
    }

    bool Camera::set(CameraProperties type, float value)
    {
        value_operator_->createValueOperator(type);
        nRet_ = value_operator_->setValue(value);
        return nRet_;
    }

    bool Camera::reset()
    {
        nRet_ = this->set(CAP_PROP_FRAMERATE_ENABLE, frame_rate_enable_)                                || nRet_;
        if(frame_rate_enable_)
            nRet_ = this->set(CAP_PROP_FRAMERATE, frame_rate_)                                          || nRet_;
        nRet_ = this->set(CAP_PROP_HEIGHT, height_)                                                     || nRet_;
        nRet_ = this->set(CAP_PROP_WIDTH, width_)                                                       || nRet_;
        nRet_ = this->set(CAP_PROP_OFFSETX, Offset_x_)                                                  || nRet_;
        nRet_ = this->set(CAP_PROP_OFFSETY, Offset_y_)                                                  || nRet_;
        nRet_ = this->set(CAP_PROP_EXPOSURE_AUTO, exposure_auto_)                                       || nRet_;
        if(!exposure_auto_)
            nRet_ = this->set(CAP_PROP_EXPOSURE_TIME, exposure_time_)                                   || nRet_;
        else {
            nRet_ = this->set(CAP_PROP_AUTO_EXPOSURE_TIME_LOWER_LIMIT, auto_exposure_time_lower_limit_) || nRet_;
            nRet_ = this->set(CAP_PROP_AUTO_EXPOSURE_TIME_UPPER_LIMIT, auto_exposure_time_upper_limit_) || nRet_;
        }
        nRet_ = this->set(CAP_PROP_BALANCE_WHITE_AUTO, balance_white_auto_)                             || nRet_;
        nRet_ = this->set(CAP_PROP_BRIGHTNESS, brightness_)                                             || nRet_;
        nRet_ = this->set(CAP_PROP_GAMMA_ENABLE, gamma_enable_)                                         || nRet_;
        if(gamma_enable_) {
            nRet_ = this->set(CAP_PROP_GAMMA_SELECTOR, gamma_selector_)                                 || nRet_;
            if(gamma_selector_ == 1)
                nRet_ = this->set(CAP_PROP_GAMMA, gamma_)                                               || nRet_;
        }
        nRet_ = this->set(CAP_PROP_GAINAUTO, gain_auto_)                                                || nRet_;
        nRet_ = this->set(CAP_PROP_PIXEL_FORMAT, pixel_format_)                                         || nRet_;
        nRet_ = this->set(CAP_PROP_TRIGGER_MODE, trigger_mode_)                                         || nRet_;
        return nRet_;
    }

    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    void Camera::ReadImg(cv::Mat &image)
    {

        pthread_mutex_lock(&mutex);
        if (frame_empty)
        {
            image = cv::Mat();
        }
        else
        {
            if(calibrate_enable_) {
                this->undistort(camera::frame, image);
            }
            else {
                image = camera::frame;
            }
            if(resize_enable_) {
                image_resize(camera::frame, image);
            }
            else {
                image = camera::frame;
            }
            frame_empty = 1;
        }
        pthread_mutex_unlock(&mutex);
    }

    void *Camera::HKWorkThread(void *p_handle)
    {
        double start;
        int nRet;
        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cv::Mat tmp;
        int image_empty_count = 0; //空图帧数
        while (ros::ok())
        {
            start = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 15);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                {
                    ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数
            //转换图像格式为BGR8

            stConvertParam.nWidth = width;                              //ch:图像宽 | en:image width
            stConvertParam.nHeight = height;                            //ch:图像高 | en:image height
            stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           //ch:输入数据大小 | en:input data size
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format                      //! 输出格式 BGR8
            stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format                       //! 输入格式 BayerRGGB8
            MV_CC_ConvertPixelType(p_handle, &stConvertParam);
            pthread_mutex_lock(&mutex);
            camera::frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone(); //tmp.clone();
            frame_empty = 0;
            pthread_mutex_unlock(&mutex);
            double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
            ROS_INFO("Grap one image time: %lf", time);
            //*************************************testing img********************************//
            // std::cout << "HK_camera,Time:" << time << "\tFPS:" << 1 / time << std::endl;
            // cv::imshow("HK vision",frame);
            // cv::waitKey(1);
        }
        free(m_pBufForDriver);
        free(m_pBufForSaveImage);
        return 0;
    }

    void Camera::undistort(cv::Mat &image_raw, cv::Mat &image_rect) {
        auto start = std::chrono::system_clock::now();
        cv::undistort(image_raw, image_rect, cameraMatrix_, distCoeffs_);
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        ROS_INFO("Undistort use time: %dms", (int)duration);
    }

    void Camera::image_resize(cv::Mat &src_image, cv::Mat &dst_image) {
        auto start = std::chrono::system_clock::now();
        cv::resize(src_image, dst_image, cv::Size(dst_height_, dst_width_), 0, 0, cv::INTER_LINEAR);
        auto end = std::chrono::system_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        ROS_INFO("Resize use time: %dms", (int)duration);
    }

    int Camera::get_width() {return width_;}
    int Camera::get_height() {return height_;}
    int Camera::get_Offset_x() {return Offset_x_;}
    int Camera::get_Offset_y() {return Offset_y_;}

    bool Camera::is_calibrate_enable() {
        return calibrate_enable_;
    }
    bool Camera::calibrate_enable() {
        calibrate_enable_ = true;
        return true;
    }
    bool Camera::calibrate_disable() {
        calibrate_enable_ = false;
        return true;
    }

    boost::array<double, 9> Camera::get_cameraMatrix() {
        boost::array<double, 9> r = {0};
        if(calibrate_enable_) {
            r.at(0) = cameraMatrix_.at<double>(0, 0);
            r.at(2) = cameraMatrix_.at<double>(0, 2);
            r.at(4) = cameraMatrix_.at<double>(1, 1);
            r.at(5) = cameraMatrix_.at<double>(1, 2);
            r.at(8) = cameraMatrix_.at<double>(2, 2);
        }
        return r;
    }

    std::vector<double> Camera::get_distCoeffs() {
        std::vector<double> r;
        if(calibrate_enable_) {
            r.push_back(distCoeffs_.at<double>(0, 0));
            r.push_back(distCoeffs_.at<double>(1, 0));
            r.push_back(distCoeffs_.at<double>(2, 0));
            r.push_back(distCoeffs_.at<double>(3, 0));
            r.push_back(distCoeffs_.at<double>(4, 0));
        }
        return r;
    }
}