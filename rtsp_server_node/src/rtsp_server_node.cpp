#include <iomanip>
#include "rtsp_server_node.h"

namespace rtsp_server_node
{
RtspServerNode::RtspServerNode(
    const std::string & node_name,
    const NodeOptions & options) : Node(node_name, options) {
  sp_rtsp_config_ = std::make_shared<RtspServerConfig>();
  std::string video_type{"h264"};

  this->declare_parameter("stream_name", sp_rtsp_config_->stream_name);
  this->declare_parameter("port", sp_rtsp_config_->port_);
  this->declare_parameter("video_type", video_type);
  this->declare_parameter("topic_name", topic_name);
  this->declare_parameter("dump_frame", dump_frame);

  this->get_parameter("stream_name", sp_rtsp_config_->stream_name);
  this->get_parameter("port", sp_rtsp_config_->port_);
  this->get_parameter("video_type", video_type);
  this->get_parameter("topic_name", topic_name);
  this->get_parameter("dump_frame", dump_frame);

  RCLCPP_WARN_STREAM(this->get_logger(),
    "\n topic_name: " << topic_name
    << "\n stream_name: " << sp_rtsp_config_->stream_name
    << "\n port: " << sp_rtsp_config_->port_
    << "\n video_type: " << video_type
    << "\n dump_frame: " << dump_frame
  );

  if (video_type == "h264") {
    sp_rtsp_config_->video_type_ = VideoType::H264;
  } else if (video_type == "h265") {
    sp_rtsp_config_->video_type_ = VideoType::H265;
  } else {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Input video_type["
      << video_type
      << "] is not supported, which should be [h264/h265]"
    );
    rclcpp::shutdown();
    return;
  }
  
  rtsp_server_ = std::make_shared<RtspServer>(sp_rtsp_config_);
  if (!rtsp_server_ || rtsp_server_->Init() != 0 || rtsp_server_->Start() != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create RtspServer");
    rclcpp::shutdown();
    return;
  }

  subscription_ = this->create_subscription<img_msgs::msg::H26XFrame>(
      topic_name, rclcpp::SensorDataQoS(),
      std::bind(&RtspServerNode::OnSubcallback, this, std::placeholders::_1));

      
  if (1 == dump_frame) {
    std::string file_name = "dump_stream.264";
    fout_.open(file_name, std::ios::out | std::ios::binary);
    RCLCPP_WARN(this->get_logger(), "dump stream to %s", file_name.c_str());
  }
}

RtspServerNode::~RtspServerNode() {
  if (1 == dump_frame && fout_.is_open()) {
    fout_.close();
  }
  rtsp_server_->Stop();
}

void RtspServerNode::OnSubcallback(img_msgs::msg::H26XFrame::ConstSharedPtr msg) {
  if (!msg) return;

  {
    auto calc_time_ms_laps = [](const struct timespec &time_start, const struct timespec &time_now)
    {
      int32_t nRetMs = 0;
      if (time_now.tv_nsec < time_start.tv_nsec)
      {
        nRetMs = (time_now.tv_sec - time_start.tv_sec - 1) * 1000 +
        (1000000000 + time_now.tv_nsec - time_start.tv_nsec) / 1000000;
      } else {
        nRetMs = (time_now.tv_sec - time_start.tv_sec) * 1000 + (time_now.tv_nsec - time_start.tv_nsec) / 1000000;
      }
      return nRetMs;
    };
    static auto tp_start = std::chrono::system_clock::now();
    static int count = 0;
    count++;
    auto tp_now = std::chrono::system_clock::now();
    if (tp_now - tp_start >= std::chrono::milliseconds(1000)) {
      struct timespec time_now = {0, 0};
      clock_gettime(CLOCK_REALTIME, &time_now);
      timespec time_start {msg->dts.sec, msg->dts.nanosec};
      auto delay_ms = calc_time_ms_laps(time_start, time_now);
      RCLCPP_WARN_STREAM(this->get_logger(),
        "recved img fps: ["
        << std::fixed << std::setprecision(2) << count / (std::chrono::duration_cast<std::chrono::milliseconds>(tp_now - tp_start).count() / 1000.0)
        << "], encoding: " << std::string((const char *)msg->encoding.data())
        << ", w: " << msg->width
        << ", h: " << msg->height
        << ", delay_ms: " << delay_ms
        );
      count = 0;
      tp_start = std::chrono::system_clock::now();
    }
  }

  rtsp_server_->SendData(msg->data.data(), msg->data.size(),
    static_cast<int>(std::string((const char *)msg->encoding.data()) == std::string("h264") ?
    rtspcomponent::VideoType::H264 : rtspcomponent::VideoType::H265)
    );

  if (1 == dump_frame && fout_.is_open()) {
    fout_.write((const char*)msg->data.data(), msg->data.size());
  }
}

}
