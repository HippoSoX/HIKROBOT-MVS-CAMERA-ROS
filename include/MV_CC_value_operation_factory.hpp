#ifndef MV_CC_VALUE_OPERATION_FACTORY
#define MV_CC_VALUE_OPERATION_FACTORY

#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include "PixelType.h"
#include <string>
#include <vector>

namespace camera {
    //********** CameraProperties config ************************************/
    enum CameraProperties
    {
        CAP_PROP_ACQUISITION_MODE,                  //采集模式
        CAP_PROP_FRAMERATE_ENABLE,                  //帧数可调
        CAP_PROP_FRAMERATE,                         //帧数
        CAP_PROP_EXPOSURE_TIME,                     //曝光时间
        CAP_PROP_EXPOSURE_AUTO,                     //自动曝光
        CAP_PROP_AUTO_EXPOSURE_TIME_LOWER_LIMIT,    //自动曝光下限
        CAP_PROP_AUTO_EXPOSURE_TIME_UPPER_LIMIT,    //自动曝光上限
        CAP_PROP_BALANCE_WHITE_AUTO,                //自动白平衡
        CAP_PROP_BRIGHTNESS,                        //亮度
        CAP_PROP_GAINAUTO,                          //自动增益
        CAP_PROP_GAMMA_ENABLE,                      //伽马因子可调
        CAP_PROP_GAMMA_SELECTOR,                    //伽马因子来源？？
        CAP_PROP_GAMMA,                             //伽马因子
        CAP_PROP_HEIGHT,                            //图像高度
        CAP_PROP_WIDTH,                             //图像宽度
        CAP_PROP_OFFSETX,                           //X偏置
        CAP_PROP_OFFSETY,                           //Y偏置
        CAP_PROP_PIXEL_FORMAT,                      //图像像素格式
        // CAP_PROP_RESULTING_FRAMERATE,               //实际帧率
        CAP_PROP_IMAGE_COMPRESSION_MODE,            
        CAP_PROP_IMAGE_COMPRESSION_QUALITY,

        CAP_PROP_TRIGGER_MODE,                      //外部触发
        // CAP_PROP_TRIGGER_SOURCE,                    //触发源
        // CAP_PROP_LINE_SELECTOR,                     //触发线
        
        NUM_OF_CAP_PROP
    };

    const std::vector<std::string> CAMERA_PROPERTIES_STRING_VECTOR = {
        "Acquisition Mode",
        "Acquisition Frame Rate Enable",
        "Acquisition Frame Rate",
        "Exposure Time",
        "Exposure Auto",
        "Auto Exposure Time Lower Limit",
        "Auto Exposure Time Upper Limit",
        "Balance White Auto",
        "Brightness",
        "Gain Auto",
        "Gamma Enable",
        "Gamma Selector",
        "Gamma",
        "Height",
        "Width",
        "Offset X",
        "Offset Y",
        "Pixel Format",
        // "Resulting Frame Rate",
        "Image Compression Mode",
        "Image Compression Quality",
        "Trigger Mode",
        // "Trigger Source",
        // "Line Selector"
    };

    //********** 图像格式 **********/
    // 0x01100003:Mono10
    // 0x010C0004:Mono10Packed
    // 0x01100005:Mono12
    // 0x010C0006:Mono12Packed
    // 0x01100007:Mono16
    // 0x02180014:RGB8Packed
    // 0x02100032:YUV422_8
    // 0x0210001F:YUV422_8_UYVY
    // 0x01080008:BayerGR8
    // 0x01080009:BayerRG8
    // 0x0108000A:BayerGB8
    // 0x0108000B:BayerBG8
    // 0x0110000e:BayerGB10
    // 0x01100012:BayerGB12
    // 0x010C002C:BayerGB12Packed
    enum PixelFormat {
        Mono1p = 0,
        Mono2p,
        Mono4p,
        Mono8 = 3,
        Mono8_Signed,
        Mono10 = 5,
        Mono10Packed = 6,
        Mono12 = 7,
        Mono12Packed = 8,
        Mono14 = 9,
        Mono16 = 10,

        BayerGR8 = 11,
        BayerRG8 = 12,
        BayerGB8 = 13,
        BayerBG8 = 14,
        BayerRBGG8 = 15,
        BayerGR10 = 16,
        BayerRG10 = 17,
        BayerGB10 = 18,
        BayerBG10 = 19,
        BayerGR12 = 20,
        BayerRG12 = 21,
        BayerGB12 = 22,
        BayerBG12 = 23,
        BayerGR10_Packed = 24,
        BayerRG10_Packed = 25,
        BayerGB10_Packed = 26,
        BayerBG10_Packed = 27,
        BayerGR12_Packed = 28,
        BayerRG12_Packed = 29,
        BayerGB12_Packed = 30,
        BayerBG12_Packed = 31,
        BayerGR16 = 32,
        BayerRG16 = 33,
        BayerGB16 = 34,
        BayerBG16 = 35,

        RGB8_Packed = 36,
        BGR8_Packed = 37,
        RGBA8_Packed = 38,
        BGRA8_Packed = 39,
        RGB10_Packed = 40,
        BGR10_Packed = 41,
        RGB12_Packed = 42,
        BGR12_Packed = 43,
        RGB16_Packed = 44,
        BGR16_Packed = 45,
        RGBA16_Packed = 46,
        BGRA16_Packed = 47,
        RGB10V1_Packed = 48,
        RGB10V2_Packed = 49,
        RGB12V1_Packed = 50,
        RGB565_Packed = 51,
        BGR565_Packed = 52,

        YUV411_Packed = 53,
        YUV422_Packed = 54,
        YUV422_YUYV_Packed = 55,
        YUV444_Packed,
        YCBCR8_CBYCR,
        YCBCR422_8,
        YCBCR422_8_CBYCRY,
        YCBCR411_8_CBYYCRYY,
        YCBCR601_8_CBYCR,
        YCBCR601_422_8,
        YCBCR601_422_8_CBYCRY,
        YCBCR601_411_8_CBYYCRYY,
        YCBCR709_8_CBYCR,
        YCBCR709_422_8,
        YCBCR709_422_8_CBYCRY,
        YCBCR709_411_8_CBYYCRYY,

        YUV420SP_NV12,
        YUV420SP_NV21,

        RGB8_Planar,
        RGB10_Planar,
        RGB12_Planar,
        RGB16_Planar,

        Jpeg,

        Coord3D_ABC32f,
        Coord3D_ABC32f_Planar,

        Coord3D_AC32f,
        COORD3D_DEPTH_PLUS_MASK,

        Coord3D_ABC32,
        Coord3D_AB32f,
        Coord3D_AB32,
        Coord3D_AC32f_64,
        Coord3D_AC32f_Planar,
        Coord3D_AC32,
        Coord3D_A32f,
        Coord3D_A32,
        Coord3D_C32f,
        Coord3D_C32,
        Coord3D_ABC16,
        Coord3D_C16,

        Float32,

        HB_Mono8,
        HB_Mono10,
        HB_Mono10_Packed,
        HB_Mono12,
        HB_Mono12_Packed,
        HB_Mono16,
        HB_BayerGR8,
        HB_BayerRG8,
        HB_BayerGB8,
        HB_BayerBG8,
        HB_BayerRBGG8,
        HB_BayerGR10,
        HB_BayerRG10,
        HB_BayerGB10,
        HB_BayerBG10,
        HB_BayerGR12,
        HB_BayerRG12,
        HB_BayerGB12,
        HB_BayerBG12,
        HB_BayerGR10_Packed,
        HB_BayerRG10_Packed,
        HB_BayerGB10_Packed,
        HB_BayerBG10_Packed,
        HB_BayerGR12_Packed,
        HB_BayerRG12_Packed,
        HB_BayerGB12_Packed,
        HB_BayerBG12_Packed,
        HB_YUV422_Packed,
        HB_YUV422_YUYV_Packed,
        HB_RGB8_Packed,
        HB_BGR8_Packed,
        HB_RGBA8_Packed,
        HB_BGRA8_Packed,
        HB_RGB16_Packed,
        HB_BGR16_Packed,
        HB_RGBA16_Packed,
        HB_BGRA16_Packed,

        MAX_PIXELFORMAT,

        Undefined           = 0xFF
    };

    const std::vector<unsigned int> PIXEL_FORMATVECTOR = {
        (uint32_t)PixelType_Gvsp_Mono1p,            // 0
        (uint32_t)PixelType_Gvsp_Mono2p,
        (uint32_t)PixelType_Gvsp_Mono4p,
        (uint32_t)PixelType_Gvsp_Mono8,
        (uint32_t)PixelType_Gvsp_Mono8_Signed,
        (uint32_t)PixelType_Gvsp_Mono10,            // 5
        (uint32_t)PixelType_Gvsp_Mono10_Packed,
        (uint32_t)PixelType_Gvsp_Mono12,
        (uint32_t)PixelType_Gvsp_Mono12_Packed,
        (uint32_t)PixelType_Gvsp_Mono14,
        (uint32_t)PixelType_Gvsp_Mono16,            // 10

        (uint32_t)PixelType_Gvsp_BayerGR8,          // 11
        (uint32_t)PixelType_Gvsp_BayerRG8,
        (uint32_t)PixelType_Gvsp_BayerGB8,
        (uint32_t)PixelType_Gvsp_BayerBG8,
        (uint32_t)PixelType_Gvsp_BayerRBGG8,        // 15
        (uint32_t)PixelType_Gvsp_BayerGR10,
        (uint32_t)PixelType_Gvsp_BayerRG10,
        (uint32_t)PixelType_Gvsp_BayerGB10,
        (uint32_t)PixelType_Gvsp_BayerBG10,
        (uint32_t)PixelType_Gvsp_BayerGR12,         // 20
        (uint32_t)PixelType_Gvsp_BayerRG12,
        (uint32_t)PixelType_Gvsp_BayerGB12,
        (uint32_t)PixelType_Gvsp_BayerBG12,
        (uint32_t)PixelType_Gvsp_BayerGR10_Packed,
        (uint32_t)PixelType_Gvsp_BayerRG10_Packed,  // 25
        (uint32_t)PixelType_Gvsp_BayerGB10_Packed,
        (uint32_t)PixelType_Gvsp_BayerBG10_Packed,
        (uint32_t)PixelType_Gvsp_BayerGR12_Packed,
        (uint32_t)PixelType_Gvsp_BayerRG12_Packed,
        (uint32_t)PixelType_Gvsp_BayerGB12_Packed,  // 30
        (uint32_t)PixelType_Gvsp_BayerBG12_Packed,
        (uint32_t)PixelType_Gvsp_BayerGR16,
        (uint32_t)PixelType_Gvsp_BayerRG16,
        (uint32_t)PixelType_Gvsp_BayerGB16,
        (uint32_t)PixelType_Gvsp_BayerBG16,         //35

        // RGB Packed buffer format defines 
        (uint32_t)PixelType_Gvsp_RGB8_Packed,       // 36
        (uint32_t)PixelType_Gvsp_BGR8_Packed,
        (uint32_t)PixelType_Gvsp_RGBA8_Packed,
        (uint32_t)PixelType_Gvsp_BGRA8_Packed,
        (uint32_t)PixelType_Gvsp_RGB10_Packed,      // 40
        (uint32_t)PixelType_Gvsp_BGR10_Packed,
        (uint32_t)PixelType_Gvsp_RGB12_Packed,
        (uint32_t)PixelType_Gvsp_BGR12_Packed,
        (uint32_t)PixelType_Gvsp_RGB16_Packed,
        (uint32_t)PixelType_Gvsp_BGR16_Packed,      // 45
        (uint32_t)PixelType_Gvsp_RGBA16_Packed,
        (uint32_t)PixelType_Gvsp_BGRA16_Packed,
        (uint32_t)PixelType_Gvsp_RGB10V1_Packed,
        (uint32_t)PixelType_Gvsp_RGB10V2_Packed,
        (uint32_t)PixelType_Gvsp_RGB12V1_Packed,    // 50
        (uint32_t)PixelType_Gvsp_RGB565_Packed,
        (uint32_t)PixelType_Gvsp_BGR565_Packed,

        // YUV Packed buffer format defines 
        (uint32_t)PixelType_Gvsp_YUV411_Packed,     // 53
        (uint32_t)PixelType_Gvsp_YUV422_Packed,
        (uint32_t)PixelType_Gvsp_YUV422_YUYV_Packed,    // 55
        (uint32_t)PixelType_Gvsp_YUV444_Packed,
        (uint32_t)PixelType_Gvsp_YCBCR8_CBYCR,
        (uint32_t)PixelType_Gvsp_YCBCR422_8,
        (uint32_t)PixelType_Gvsp_YCBCR422_8_CBYCRY,
        (uint32_t)PixelType_Gvsp_YCBCR411_8_CBYYCRYY,   // 60
        (uint32_t)PixelType_Gvsp_YCBCR601_8_CBYCR,
        (uint32_t)PixelType_Gvsp_YCBCR601_422_8,
        (uint32_t)PixelType_Gvsp_YCBCR601_422_8_CBYCRY,
        (uint32_t)PixelType_Gvsp_YCBCR601_411_8_CBYYCRYY,
        (uint32_t)PixelType_Gvsp_YCBCR709_8_CBYCR,      // 65
        (uint32_t)PixelType_Gvsp_YCBCR709_422_8,
        (uint32_t)PixelType_Gvsp_YCBCR709_422_8_CBYCRY,
        (uint32_t)PixelType_Gvsp_YCBCR709_411_8_CBYYCRYY,   // 68

        // YUV420
        (uint32_t)PixelType_Gvsp_YUV420SP_NV12,     // 69
        (uint32_t)PixelType_Gvsp_YUV420SP_NV21,     // 70

        // RGB Planar buffer format defines 
        (uint32_t)PixelType_Gvsp_RGB8_Planar,       // 71
        (uint32_t)PixelType_Gvsp_RGB10_Planar,
        (uint32_t)PixelType_Gvsp_RGB12_Planar,
        (uint32_t)PixelType_Gvsp_RGB16_Planar,

        // 自定义的图片格式
        (uint32_t)PixelType_Gvsp_Jpeg,              // 75

        (uint32_t)PixelType_Gvsp_Coord3D_ABC32f,    // 76 //0x026000C0
        (uint32_t)PixelType_Gvsp_Coord3D_ABC32f_Planar,     // 77 //0x026000C1

        // 该值被废弃，请参考PixelType_Gvsp_Coord3D_AC32f_64; the value is discarded
        (uint32_t)PixelType_Gvsp_Coord3D_AC32f,     // 78
        // 该值被废弃; the value is discarded    (已放入Chunkdata)
        (uint32_t)PixelType_Gvsp_COORD3D_DEPTH_PLUS_MASK,   // 79

        (uint32_t)PixelType_Gvsp_Coord3D_ABC32,     // 80 //0x82603001
        (uint32_t)PixelType_Gvsp_Coord3D_AB32f,     //0x82403002
        (uint32_t)PixelType_Gvsp_Coord3D_AB32,      //0x82403003
        (uint32_t)PixelType_Gvsp_Coord3D_AC32f_64,  //0x024000C2
        (uint32_t)PixelType_Gvsp_Coord3D_AC32f_Planar,  //0x024000C3
        (uint32_t)PixelType_Gvsp_Coord3D_AC32,      // 85 //0x82403004
        (uint32_t)PixelType_Gvsp_Coord3D_A32f,      //0x012000BD
        (uint32_t)PixelType_Gvsp_Coord3D_A32,       //0x81203005
        (uint32_t)PixelType_Gvsp_Coord3D_C32f,      //0x012000BF
        (uint32_t)PixelType_Gvsp_Coord3D_C32,       //0x81203006
        (uint32_t)PixelType_Gvsp_Coord3D_ABC16,     // 90 //0x023000B9
        (uint32_t)PixelType_Gvsp_Coord3D_C16,       //0x011000B8

        (uint32_t)PixelType_Gvsp_Float32,           // 92 //0x81200001

        //无损压缩像素格式定义
        (uint32_t)PixelType_Gvsp_HB_Mono8,          // 93
        (uint32_t)PixelType_Gvsp_HB_Mono10,
        (uint32_t)PixelType_Gvsp_HB_Mono10_Packed,  // 95
        (uint32_t)PixelType_Gvsp_HB_Mono12,
        (uint32_t)PixelType_Gvsp_HB_Mono12_Packed,
        (uint32_t)PixelType_Gvsp_HB_Mono16,
        (uint32_t)PixelType_Gvsp_HB_BayerGR8,
        (uint32_t)PixelType_Gvsp_HB_BayerRG8,       // 100
        (uint32_t)PixelType_Gvsp_HB_BayerGB8,
        (uint32_t)PixelType_Gvsp_HB_BayerBG8,
        (uint32_t)PixelType_Gvsp_HB_BayerRBGG8,
        (uint32_t)PixelType_Gvsp_HB_BayerGR10,
        (uint32_t)PixelType_Gvsp_HB_BayerRG10,      // 105
        (uint32_t)PixelType_Gvsp_HB_BayerGB10,
        (uint32_t)PixelType_Gvsp_HB_BayerBG10,
        (uint32_t)PixelType_Gvsp_HB_BayerGR12,
        (uint32_t)PixelType_Gvsp_HB_BayerRG12,
        (uint32_t)PixelType_Gvsp_HB_BayerGB12,      // 110
        (uint32_t)PixelType_Gvsp_HB_BayerBG12,
        (uint32_t)PixelType_Gvsp_HB_BayerGR10_Packed,
        (uint32_t)PixelType_Gvsp_HB_BayerRG10_Packed,
        (uint32_t)PixelType_Gvsp_HB_BayerGB10_Packed,
        (uint32_t)PixelType_Gvsp_HB_BayerBG10_Packed,   // 115
        (uint32_t)PixelType_Gvsp_HB_BayerGR12_Packed,
        (uint32_t)PixelType_Gvsp_HB_BayerRG12_Packed,
        (uint32_t)PixelType_Gvsp_HB_BayerGB12_Packed,
        (uint32_t)PixelType_Gvsp_HB_BayerBG12_Packed,
        (uint32_t)PixelType_Gvsp_HB_YUV422_Packed,      // 120
        (uint32_t)PixelType_Gvsp_HB_YUV422_YUYV_Packed,
        (uint32_t)PixelType_Gvsp_HB_RGB8_Packed,
        (uint32_t)PixelType_Gvsp_HB_BGR8_Packed,
        (uint32_t)PixelType_Gvsp_HB_RGBA8_Packed,
        (uint32_t)PixelType_Gvsp_HB_BGRA8_Packed,   // 125
        (uint32_t)PixelType_Gvsp_HB_RGB16_Packed,
        (uint32_t)PixelType_Gvsp_HB_BGR16_Packed,
        (uint32_t)PixelType_Gvsp_HB_RGBA16_Packed,
        (uint32_t)PixelType_Gvsp_HB_BGRA16_Packed,  // 129

        (uint32_t)PixelType_Gvsp_Undefined
    };

    class MVCCValue {
        public:
            virtual int setValue(float) = 0;
            // virtual void getValue() = 0;
        protected:
            void *handle_;
    };

    class MVCCAcquisitionMode: public MVCCValue {
        public:
            MVCCAcquisitionMode(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCAcquisitionMode();

            int setValue(float value) {
                return MV_CC_SetAcquisitionMode(this->handle_, (unsigned int)value);
            }
    };

    class MVCCFrameRateEnable: public MVCCValue {
        public:
            MVCCFrameRateEnable(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCFrameRateEnable();

            int setValue(float value) {
                return MV_CC_SetBoolValue(this->handle_, "AcquisitionFrameRateEnable", (bool)value);
            }
    };

    class MVCCFrameRate: public MVCCValue {
        public:
            MVCCFrameRate(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCFrameRate();

            int setValue(float value) {
                return MV_CC_SetFrameRate(handle_, value);
            }
    };

    class MVCCExposureTime: public MVCCValue {
        public:
            MVCCExposureTime(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCExposureTime();

            int setValue(float value) {
                return MV_CC_SetExposureTime(handle_, value);
            }
    };

    class MVCCExposureAuto: public MVCCValue {
        public:
            MVCCExposureAuto(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCExposureAuto();

            int setValue(float value) {
                return MV_CC_SetExposureAutoMode(handle_, (unsigned int)value);
            }
    };

    class MVCCAutoExposureTimeLowerLimit: public MVCCValue {
        public:
            MVCCAutoExposureTimeLowerLimit(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCAutoExposureTimeLowerLimit();

            int setValue(float value) {
                return MV_CC_SetAutoExposureTimeLower(handle_, (unsigned int)value);
            }
    };

    class MVCCAutoExposureTimeUpperLimit: public MVCCValue {
        public:
            MVCCAutoExposureTimeUpperLimit(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCAutoExposureTimeUpperLimit();

            int setValue(float value) {
                return MV_CC_SetAutoExposureTimeUpper(handle_, (unsigned int)value);
            }
    };

    class MVCCBalanceWhiteAuto: public MVCCValue {
        public:
            MVCCBalanceWhiteAuto(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCBalanceWhiteAuto();

            int setValue(float value) {
                return MV_CC_SetBalanceWhiteAuto(handle_, (unsigned int)value);
            }
    };

    class MVCCBrightness: public MVCCValue {
        public:
            MVCCBrightness(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCBrightness();

            int setValue(float value) {
                return MV_CC_SetBrightness(handle_, (unsigned int)value);
            }
    };

    class MVCCGainAuto: public MVCCValue {
        public:
            MVCCGainAuto(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCGainAuto();

            int setValue(float value) {
                return MV_CC_SetGainMode(handle_, (unsigned int)value);
            }
    };

    class MVCCGammaEnable: public MVCCValue {
        public:
            MVCCGammaEnable(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCGammaEnable();

            int setValue(float value) {
                return MV_CC_SetBoolValue(handle_, "GammaEnable", (unsigned int)value);
            }
    };

    class MVCCGammaSelector: public MVCCValue {
        public:
            MVCCGammaSelector(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCGammaSelector();

            int setValue(float value) {
                return MV_CC_SetGammaSelector(handle_, (unsigned int)value);
            }
    };

    class MVCCGamma: public MVCCValue {
        public:
            MVCCGamma(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCGamma();

            int setValue(float value) {
                return MV_CC_SetGamma(handle_, value);
            }
    };

    class MVCCHeight: public MVCCValue {
        public:
            MVCCHeight(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCHeight();

            int setValue(float value) {
                return MV_CC_SetHeight(handle_, (unsigned int)value);
            }
    };

    class MVCCWidth: public MVCCValue {
        public:
            MVCCWidth(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCWidth();

            int setValue(float value) {
                return MV_CC_SetWidth(handle_, (unsigned int)value);
            }
    };

    class MVCCOffsetX: public MVCCValue {
        public:
            MVCCOffsetX(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCOffsetX();

            int setValue(float value) {
                return MV_CC_SetAOIoffsetX(handle_, (unsigned int)value);
            }
    };

    class MVCCOffsetY: public MVCCValue {
        public:
            MVCCOffsetY(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCOffsetY();

            int setValue(float value) {
                return MV_CC_SetAOIoffsetY(handle_, (unsigned int)value);
            }
    };

    class MVCCPixelFormat: public MVCCValue {
        public:
            MVCCPixelFormat(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCPixelFormat();

            int setValue(float value) {
                if(value >= MAX_PIXELFORMAT) {
                    return MV_E_GC_RANGE;
                }
                return MV_CC_SetPixelFormat(handle_, PIXEL_FORMATVECTOR.at(value));
            }
    };

    // class MVCCResultingFrameRate: public MVCCValue {
    //     public:
    //         MVCCResultingFrameRate(void* handle) {
    //             this->handle_ = handle;
    //         }
    //         ~MVCCResultingFrameRate();

    //         int getValue(float value) {}
    // };

    class MVCCTriggerMode: public MVCCValue {
        public:
            MVCCTriggerMode(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCTriggerMode();

            int setValue(float value) {
                return MV_CC_SetTriggerMode(handle_, value);
            }
    };

    // class MVCCTriggerSource: public MVCCValue {
    //     public:
    //         MVCCTriggerSource(void* handle) {
    //             this->handle_ = handle;
    //         }
    //         ~MVCCTriggerSource();

    //         int setValue(float value) {
    //             return MV_CC_SetTriggerSource(handle_, value);
    //         }
    // };

    // class MVCCLineSelector: public MVCCValue {
    //     public:
    //         MVboolCCLineSelector(void* handle) {
    //             this->handle_ = handle;
    //         }
    //         ~MVCCLineSelector();

    //         int setValue(float value) {
    //             return MV_CC_SetEnumValue(handle_, "LineSelector", value);
    //         }
    // };

    class MVCCImageCompressionMode: public MVCCValue {
        public:
            MVCCImageCompressionMode(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCImageCompressionMode();

            int setValue(float value) {
                return MV_CC_SetEnumValue(handle_, "ImageCompressionMode", (unsigned int)value);
            }
    };

    class MVCCImageCompressionQuality: public MVCCValue {
        public:
            MVCCImageCompressionQuality(void* handle) {
                this->handle_ = handle;
            }
            ~MVCCImageCompressionQuality();

            int setValue(float value) {
                return MV_CC_SetIntValue(handle_, "ImageCompressionQuality", (unsigned int)value);
            }
    };

    class MVCCValueOperationFactory {
        public:
            MVCCValueOperationFactory(void **handle) {
                this->handle_ = *handle;
            }
            ~MVCCValueOperationFactory() {
                releaseValuwOperator();
            }

            bool createValueOperator(CameraProperties type) {
                releaseValuwOperator();
                this->type_ = type;
                switch(type_){
                    case CAP_PROP_ACQUISITION_MODE:
                        value_operator_ = new MVCCAcquisitionMode(handle_);
                        break;
                    case CAP_PROP_FRAMERATE_ENABLE:
                        value_operator_ = new MVCCFrameRateEnable(handle_);
                        break;
                    case CAP_PROP_FRAMERATE:
                        value_operator_ = new MVCCFrameRate(handle_);
                        break;
                    case CAP_PROP_EXPOSURE_TIME:
                        value_operator_ = new MVCCExposureTime(handle_);
                        break;
                    case CAP_PROP_EXPOSURE_AUTO:
                        value_operator_ = new MVCCExposureAuto(handle_);
                        break;
                    case CAP_PROP_AUTO_EXPOSURE_TIME_LOWER_LIMIT:
                        value_operator_ = new MVCCAutoExposureTimeLowerLimit(handle_);
                        break;
                    case CAP_PROP_AUTO_EXPOSURE_TIME_UPPER_LIMIT:
                        value_operator_ = new MVCCAutoExposureTimeUpperLimit(handle_);
                        break;
                    case CAP_PROP_BALANCE_WHITE_AUTO:
                        value_operator_ = new MVCCBalanceWhiteAuto(handle_);
                        break;
                    case CAP_PROP_BRIGHTNESS:
                        value_operator_ = new MVCCBrightness(handle_);
                        break;
                    case CAP_PROP_GAMMA_ENABLE:
                        value_operator_ = new MVCCGammaEnable(handle_);
                        break;
                    case CAP_PROP_GAMMA_SELECTOR:
                        value_operator_ = new MVCCGammaSelector(handle_);
                        break;
                    case CAP_PROP_GAMMA:
                        value_operator_ = new MVCCGamma(handle_);
                        break;
                    case CAP_PROP_GAINAUTO:
                        value_operator_ = new MVCCGainAuto(handle_);
                        break;
                    case CAP_PROP_HEIGHT:
                        value_operator_ = new MVCCHeight(handle_);
                        break;
                    case CAP_PROP_WIDTH:
                        value_operator_ = new MVCCWidth(handle_);
                        break;
                    case CAP_PROP_OFFSETX:
                        value_operator_ = new MVCCOffsetX(handle_);
                        break;
                    case CAP_PROP_OFFSETY:
                        value_operator_ = new MVCCOffsetY(handle_);
                        break;
                    case CAP_PROP_PIXEL_FORMAT:
                        value_operator_ = new MVCCPixelFormat(handle_);
                        break;
                    // case CAP_PROP_RESULTING_FRAMERATE:
                    //     value_operator_ = new MVCCResultingFrameRate(handle_);
                    //     break;
                    case CAP_PROP_TRIGGER_MODE:
                        value_operator_ = new MVCCTriggerMode(handle_);
                        break;
                    // case CAP_PROP_TRIGGER_SOURCE:
                    //     value_operator_ = new MVCCTriggerSource(handle_);
                    //     break;
                    // case CAP_PROP_LINE_SELECTOR:
                    //     value_operator_ = new MVCCLineSelector(handle_);
                    //     break;
                    case CAP_PROP_IMAGE_COMPRESSION_MODE:
                        value_operator_ = new MVCCImageCompressionMode(handle_);
                        break;
                    case CAP_PROP_IMAGE_COMPRESSION_QUALITY:
                        value_operator_ = new MVCCImageCompressionQuality(handle_);
                        break;
                    default:
                        break;
                }
                return true;
            }

            void releaseValuwOperator() {
                if(value_operator_ != NULL) {
                    delete value_operator_;
                    value_operator_ = NULL;
                }
                return;
            }

            bool setValue(float value) {
                if(value_operator_ == NULL) {
                    return false;
                }
                int ret = value_operator_->setValue(value);
                if(ret == MV_OK) {
                    ROS_INFO("Set %s OK! value=%lf\n", CAMERA_PROPERTIES_STRING_VECTOR.at(type_).c_str(), value);
                }
                else {
                    ROS_WARN("Set %s failed! error code=0x%x\n", CAMERA_PROPERTIES_STRING_VECTOR.at(type_).c_str(), ret);
                }
                return ret;
            }
            
            bool getValue(float value);
        
        private:
            void *handle_;
            MVCCValue* value_operator_ = NULL;
            CameraProperties type_;
    };
}

#endif

// int main(int argc, char **argv) {
//     void *handle;
//     camera::CameraProperties type = camera::CAP_PROP_FRAMERATE;
//     camera::MVCCValueOperationFactory value_operator(handle);
//     float value = 0.;
//     value_operator.createValueOperator(type);
//     value_operator.setValue(value);
//     return 0;
// }
