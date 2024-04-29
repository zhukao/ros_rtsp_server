#include "rclcpp/rclcpp.hpp"
#include "img_msgs/msg/h26_x_frame.hpp"
#include "rtsp_server/rtsp_server.h"

namespace rtsp_server_node
{
using rclcpp::NodeOptions;
using rtspcomponent::RtspServer;
using rtspcomponent::RtspServerConfig;
using rtspcomponent::VideoType;

class RtspServerNode : public rclcpp::Node
{
public:
  RtspServerNode(
    const std::string & node_name,
    const NodeOptions & options = NodeOptions());
  ~RtspServerNode();

private:
  std::shared_ptr<RtspServerConfig> sp_rtsp_config_= nullptr;
  std::shared_ptr<RtspServer> rtsp_server_ = nullptr;

  std::string topic_name = "video_data";
  rclcpp::Subscription<img_msgs::msg::H26XFrame>::SharedPtr subscription_ = nullptr;
  void OnSubcallback(img_msgs::msg::H26XFrame::ConstSharedPtr msg);

  int dump_frame = 0;
};
}