# Bug record

## 1.image_transport无法发出compressed图像

报错信息如下

```
/home/hippo/catkin_ws/devel/lib/hikrobot_camera/hikrobot_camera: symbol lookup error: /opt/ros/melodic/lib//libcompressed_image_transport.so: undefined symbol: _ZN2cv6String8allocateEm
```

查到的信息：

```
ldd -r libcompressed_image_transport.so
...
undefined symbol: _ZN2cv6String8allocateEm	(./libcompressed_image_transport.so)

c++filt _ZN2cv6String8allocateEm
cv::String::allocate(unsigned long)
```
