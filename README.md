# ros_rtsp_server

ros_rtsp_server是基于live555框架开发的ros2功能包，作为rtsp服务端，用于通过rtsp推送H264/H265编码视频，客户端可通过url来拉取视频。

# 支持的平台和系统

- 平台：X86, ARM
- 系统：Ubuntu 20.04 & ROS2 Foxy, Ubuntu 22.04 & ROS2 Humble

# 安装依赖

安装[LIVE555 Media Server](http://www.live555.com/mediaServer/)。

## Ubuntu 20.04

直接使用apt安装：
```bash
sudo apt install liblivemedia-dev
```

## Ubuntu 22.04

下载源码编译安装：
```bash
# 下载源码
git clone https://github.com/rgaufman/live555.git
# 编译
./genMakefiles linux
make -j4
# 安装在/usr/local/路径
sudo make install
```

# 编译和运行

## 推流本地编码视频

通过ffmpeg读取本地编码视频文件后，通过rtsp推流。

### X86平台

- 编译
  
```bash
# 下载源码
git clone https://github.com/HorizonRDK/hobot_msgs.git
git clone xxx
# 编译
colcon build --packages-up-to ros_rtsp_server
```

- 运行

```bash
source install/local_setup.bash
export DATA_SRC=feedback
ros2 run ros_rtsp_server ros_rtsp_server `ros2 pkg prefix ros_rtsp_server`/lib/ros_rtsp_server/data/chn0.264 10.64.29.52 8003
```

### ARM平台

```bash
export DATA_SRC=feedback
ros2 run ros_rtsp_server ros_rtsp_server `ros2 pkg prefix ros_rtsp_server`/lib/ros_rtsp_server/data/chn0.264 10.64.29.52 8003
```



## 订阅工具发布的本地编码视频

使用`hobot_image_publisher`工具读取本地编码视频文件并发布ros2 topic，订阅码流消息后通过rtsp推流。

### X86平台

仅限于**Ubuntu 22.04 & ROS2 Humble**。

- 编译
  
```bash
# 安装依赖
sudo apt install liblivemedia-dev libjsoncpp-dev
# 下载源码
git clone https://github.com/HorizonRDK/hobot_image_publisher.git
git clone https://github.com/HorizonRDK/hobot_msgs.git
git clone xxx
# 编译
colcon build --packages-up-to ros_rtsp_server hobot_image_publisher --cmake-args -DPLATFORM_X86=ON
```

- 运行

```bash
source install/local_setup.bash
# 终端1，读取并发布码流
ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.h264 -p fps:=30 -p image_format:=h264 -p is_shared_mem:=False -p msg_pub_topic_name:=h264
# 终端2，订阅码流并推流
export DATA_SRC=sub
ros2 run ros_rtsp_server ros_rtsp_server --ros-args -p topic_name:=/h264 -p video_type:=h264 -p port:=8003 -p ip:=10.64.29.52
```

### ARM平台



## 订阅codec发布的实时编码视频

在[RDK平台](https://developer.horizon.cc/documents_tros/)上，使用mipi摄像头采集图像，通过codec编码发布ros2 topic，订阅码流消息后通过rtsp推流。

- 安装TogetheROS.Bot

  参考[TogetheROS.Bot用户手册](https://developer.horizon.cc/documents_tros/quick_start/install_tros)。

- 编译
  
```bash
sudo apt install liblivemedia-dev libjsoncpp-dev

git clone xxx -b develop

colcon build --packages-up-to ros_rtsp_server hobot_image_publisher
```

- 运行
  
```bash
ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37

ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=h264 codec_sub_topic:=/hbmem_img codec_pub_topic:=/h264


export DATA_SRC=sub
ros2 run ros_rtsp_server ros_rtsp_server --ros-args -p topic_name:=/h264
```

# 展示

使用[VLC media player](https://www.videolan.org/vlc/)播放码流。
