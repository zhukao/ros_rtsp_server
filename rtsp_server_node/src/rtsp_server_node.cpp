#include "rtsp_server_node.h"
// #include "rtsp_server_config.h"

namespace rtsp_server_node
{
RtspServerNode::RtspServerNode(
    const std::string & node_name,
    const NodeOptions & options) : Node(node_name, options) {
  sp_rtsp_config_ = std::make_shared<RtspServerConfig>();
  std::string video_type{""};

  this->declare_parameter("stream_name", sp_rtsp_config_->stream_name);
  this->declare_parameter("port", sp_rtsp_config_->port_);
  this->declare_parameter("ip", sp_rtsp_config_->ip_);
  this->declare_parameter("video_type", video_type);
  this->declare_parameter("topic_name", topic_name);
  this->declare_parameter("dump_frame", dump_frame);

  this->get_parameter("stream_name", sp_rtsp_config_->stream_name);
  this->get_parameter("port", sp_rtsp_config_->port_);
  this->get_parameter("ip", sp_rtsp_config_->ip_);
  this->get_parameter("video_type", video_type);
  this->get_parameter("topic_name", topic_name);
  this->get_parameter("dump_frame", dump_frame);

  RCLCPP_WARN_STREAM(this->get_logger(),
    "\n topic_name: " << topic_name
    << "\n stream_name: " << sp_rtsp_config_->stream_name
    << "\n port: " << sp_rtsp_config_->port_
    << "\n ip: " << sp_rtsp_config_->ip_
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
}

RtspServerNode::~RtspServerNode() {
  rtsp_server_->Stop();
}

void RtspServerNode::OnSubcallback(img_msgs::msg::H26XFrame::ConstSharedPtr msg) {
  if (!msg) return;

  rtsp_server_->SendData(msg->data.data(), msg->data.size(),
    static_cast<int>(std::string((const char *)msg->encoding.data()) == std::string("h264") ?
    rtspcomponent::VideoType::H264 : rtspcomponent::VideoType::H265)
    );

  if (1 == dump_frame) {
    std::string file_name = "dump_stream.264";
    std::fstream fout(file_name, std::ios::out |
                      std::ios::binary | std::ios::app);
    fout.write((const char*)msg->data.data(), msg->data.size());
    fout.close();
  }
}

}
