#ifndef MV_CC_VALUE_OPERATION_FACTORY
#define MV_CC_VALUE_OPERATION_FACTORY

#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
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
        BayerRG8 = 0,
    };

    const std::vector<unsigned int> PIXEL_FORMATVECTOR = {
        0x01080009
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