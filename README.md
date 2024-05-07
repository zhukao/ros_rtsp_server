# ros_rtsp_server

ros_rtsp_server是基于live555框架开发的ros2功能包，作为rtsp服务端，用于通过rtsp推送H264/H265编码视频，客户端可通过url来拉取视频。

# 支持的平台和系统

- 平台：X86, ARM
- 系统：Ubuntu 20.04 & ROS2 Foxy, Ubuntu 22.04 & ROS2 Humble

# 安装依赖

安装编译和运行时依赖[LIVE555 Media Server](http://www.live555.com/mediaServer/)，具体命令如下：

## Ubuntu 20.04

直接使用apt命令安装：
```bash
sudo apt install liblivemedia-dev
```

## Ubuntu 22.04

需要下载源码编译后安装：
```bash
# 下载源码
git clone https://github.com/rgaufman/live555.git
cd live555
# 修改脚本，编译动态库，默认编译静态库
sed -i "s/\$platform/\${platform}-with-shared-libraries/" genMakefiles
# 编译
./genMakefiles linux
make -j4
# 安装在/usr/local/路径
sudo make install
```

# 编译和运行

## 功能1：推流本地编码视频

使用ffmpeg读取本地编码视频文件后，通过rtsp推流。

- 编译
  
```bash
# 下载源码
git clone https://github.com/HorizonRDK/hobot_msgs.git
git clone https://github.com/zhukao/ros_rtsp_server.git
# 编译
colcon build --packages-up-to ros_rtsp_server
```

- 运行

```bash
source install/local_setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/
export DATA_SRC=feedback
ros2 run ros_rtsp_server ros_rtsp_server `ros2 pkg prefix ros_rtsp_server`/lib/ros_rtsp_server/data/chn0.264 127.0.0.1 8003
```

## 功能2：推流工具发布的本地编码视频

仅限于**Ubuntu 22.04 & ROS2 Humble**。

使用`hobot_image_publisher`工具读取本地编码视频文件并发布ros2 topic，订阅码流消息后通过rtsp推流。

- 编译
  
```bash
# 下载源码
git clone https://github.com/HorizonRDK/hobot_image_publisher.git
git clone https://github.com/HorizonRDK/hobot_msgs.git
git clone https://github.com/zhukao/ros_rtsp_server.git
# 编译
colcon build --packages-up-to ros_rtsp_server hobot_image_publisher
```

- 运行

```bash
# 终端1，读取并发布码流
source install/local_setup.bash
cp -r `ros2 pkg prefix hobot_image_publisher`/lib/hobot_image_publisher/config/ .

ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config/test1.h264 -p fps:=30 -p image_format:=h264 -p is_shared_mem:=False -p msg_pub_topic_name:=h264

# 终端2，订阅码流并推流
source install/local_setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/
export DATA_SRC=sub
ros2 run ros_rtsp_server ros_rtsp_server --ros-args -p topic_name:=/h264 -p video_type:=h264 -p port:=8003
```

## 功能3：推流摄像头采集的实时图片

仅限于**RDK平台**。

在[RDK平台](https://developer.horizon.cc/documents_tros/)上，使用mipi摄像头采集图像，通过codec编码发布ros2 topic，订阅码流消息后通过rtsp推流。

- 安装TogetheROS.Bot

  参考[TogetheROS.Bot用户手册](https://developer.horizon.cc/documents_tros/quick_start/install_tros)。

- 编译
  
```bash
git clone https://github.com/zhukao/ros_rtsp_server.git -b develop
colcon build --packages-up-to ros_rtsp_server
```

- 运行
  
```bash

# 终端1，使用mipi摄像头采集图像数据
source install/local_setup.bash
ros2 launch mipi_cam mipi_cam.launch.py mipi_video_device:=F37

# 终端2，启动编码
source install/local_setup.bash
ros2 launch hobot_codec hobot_codec.launch.py codec_in_mode:=shared_mem codec_in_format:=nv12 codec_out_mode:=ros codec_out_format:=h264 codec_sub_topic:=/hbmem_img codec_pub_topic:=/h264

# 终端3，启动推流
source install/local_setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/
export DATA_SRC=sub
ros2 run ros_rtsp_server ros_rtsp_server --ros-args -p topic_name:=/h264 -p video_type:=h264 -p port:=8003
```

# 拉流展示

启动`ros_rtsp_server`后输出的log中，`Play this stream using the URL`字段会提示播放地址，如`rtsp://127.0.0.1:8003/chn0`。

使用此url和[VLC media player](https://www.videolan.org/vlc/)播放码流。

注意，在VLC的**打开媒体**窗口中可以通过设置缓存时间降低延时，默认1000ms。


# 参数

| 名称           | 参数值                                          | 说明                                               |
| ---------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| stream_name   | "chn0"（默认）                      | 创建rtsp server的streamName，用于区分多路流  |
| port          | 555（默认）                         | 创建rtsp server的端口号   |
| video_type    | "h264"（默认）<br />"h264"/"h265"  | 图像编码方式      |
| topic_name    | "video_data"                        | 订阅的编码数据话题名  |
| dump_frame    | 0（默认，不保存）<br />0/1             | 是否将订阅到的编码数据保存到本地，如保存，在运行路径下将数据保存到dump_stream.264文件  |

# 常见问题

