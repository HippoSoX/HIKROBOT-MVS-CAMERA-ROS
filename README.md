# hikrobot_camera

此包为海康机器人工业相机的ros驱动包，在海康官方SDK的基础上进行封装与二次开发，支持USB与GigE接口的设备。

使用前请安装[MVS](https://www.hikrobotics.com/cn/machinevision/service/download?module=0)。

SDK版本：v4.1.0 （适配性未经验证，有任何问题欢迎提交）

# config

## calibration.yaml

相机内参和畸变校正参数。

## camera.yaml

通过SDK开放的可调参数接口。

| 参数名 | 默认值 | 备注 |
|--|--|--|
| width | 1440 | 图像宽度|
| height | 540 | 图像高度 |
| Offset_x | 0 | X方向偏置 |
| Offset_Y | 540 | Y方向偏置 |
| PixelFormat | 10 | 像素格式,enum类型。目前已支持标准库中部分常用类型，包括Mono、Bayer、RGB packed、YUV Packed等<br>相机具体支持的像素格式类型请参考产品用户手册<br>常用的类型的取值为: <br>3: Mono8<br>5: Mono10<br>6: Mono10Packed<br>7: Mono12<br>8: Mono12Packed<br>10: Mono16<br>36: RGB8_Packed<br>37: BGR8_Packed<br>54: YUV422_Packed<br>55: YUV422_YUYV_Packed<br>11: BayerGR8<br>12: BayerRG8<br>13: BayerGB8<br>14: BayerBG8<br>18: BayerGB10<br>26: BayerGB10_Packed<br>22: BayerGB12<br>30: BayerGB12_Packed<br>更多取值参考标准库头文件`PixelType.h`以及`./include/MV_CC_value_operation_factory.hpp:86~384` |
| FrameRateEnable | true | 使能帧率控制 |
| FrameRate | 30 | 帧率 |
| ExposureTime | 2000 | 曝光时间(us)<br>经过实测，白天在户外光照充足的情况下，2ms可清晰成像<br>晚上在车间2楼需要提高到5ms<br>推测阴天需在2～5ms之间调节 |
| ExposureAuto | false | 允许自动曝光，由硬件内部自动实现，此时ExposureTime设置失效，其值会由硬件内部改变 |
| AutoExposureTimeLowerLimit | 100 | 自动曝光下限 |
| AutoExposureTimeUpperLimit | 16777 | 自动曝光上限，此值基本上决定了帧率。要达到最大帧率60fps，曝光时间需在17ms以下 |
| BalanceWhiteAuto | 1 | 自动白平衡<br>0: Off<br>2: Once<br>1: Continuous |
| Brightness | 100 | 亮度，取值为0～255 |
| GammaEnable | true | 允许伽马增益 |
| GammaSelector | 1 | 伽马增益来源？基函数？<br>1: User 此时伽马因子生效<br>2: sRGB |
| Gamma | 0.7 | 伽马因子 当且仅当GammaEnable==True && GammaSelector==1时生效 |
| GainAuto | 2 | 亮度的固定增益<br>0: Off<br>1: Once<br>2: Continuous |
| CalibrateEnable | false | 开启图像畸变校正。非硬件实现，由opencv实现，速度较慢 |
| TriggerMode | 0 | 触发模式，关闭时会自动推送图像流，开启时需实现软触发或硬触发<br>0: Off<br>1: On |
| ResizeEnable | false | Resize image 通过线性插值（而非裁剪）的方法压缩图片 |
| DestinationHeight | 640 |  |
| DestinationWidth | 640 | |

# run & launch

```
roslaunch hikrobot_camera hikrobot_camera.launch
```

# topics

此包发布的话题有:

```
/hikrobot_camera/rgb                    type: sensor_msgs::Image
/hikrobot_camera/camera_info            type: sensor_msgs::CameraInfo
```