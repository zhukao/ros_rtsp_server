#include <iostream>
#include <signal.h>
#include "media_producer.hpp"
#include "rtsp_server_node.h"

static bool exit_ = false;

static void signal_handle(int param) {
  std::cout << "recv signal " << param << ", stop" << std::endl;
  if (param == SIGINT) {
    exit_ = true;
  }
}

int main(int argc, char **argv) {
  std::string input_data_src{"sub"};
  char* input_data_src_env = getenv("DATA_SRC");
  if (input_data_src_env) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("ros_rtsp_server"),
      "Input DATA_SRC[sub/feedback] env: " << input_data_src_env
      );
    if (std::string(input_data_src_env) == "sub" || std::string(input_data_src_env) == "feedback") {
      input_data_src = std::string(input_data_src_env);
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("ros_rtsp_server"),
        "Invalid DATA_SRC[sub/feedback] env: " << input_data_src_env
        );
      return -1;
    }
  } else {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("ros_rtsp_server"),
      "DATA_SRC[sub/feedback] env is not set, using default: " << input_data_src
      );
  }
  RCLCPP_WARN_STREAM(rclcpp::get_logger("ros_rtsp_server"),
    "input_data_src is " << input_data_src
    );

  if (input_data_src == "sub") {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rtsp_server_node::RtspServerNode>(
      "rtsp_server_node");
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
  }

  if (argc < 4) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ros_rtsp_server"),
      "usage: ros2 run ros_rtsp_server " << 
      std::string(argv[0]).substr(std::string(argv[0]).find_last_of("/") + 1) <<
      " <video_path> <ip> <port>");
    return -1;
  }

  std::string video_path{argv[1]};
  std::string ip{argv[2]};
  int port = atoi(argv[3]);

  RCLCPP_WARN_STREAM(rclcpp::get_logger("ros_rtsp_server"),
    "\n video_path is " << video_path
    << "\n ip is " << ip
    << "\n port is " << port
  );
  
  signal(SIGINT, signal_handle);
  signal(SIGPIPE, signal_handle);
  signal(SIGSEGV, signal_handle);

  auto sp_rtsp_config = std::make_shared<rtspcomponent::RtspServerConfig>();
  sp_rtsp_config->ip_ = ip;
  sp_rtsp_config->port_ = port;

  std::shared_ptr<MediaProducer> media_producer =
    std::make_shared<MediaProducer>(sp_rtsp_config, video_path);

  auto ret = media_producer->Init();
  if (ret != 0) {
    std::cerr << "rtsp server init failed" << std::endl;
    return -1;
  }

  ret = media_producer->Start();
  if (ret != 0) {
    std::cerr << "rtsp server start failed" << std::endl;
    return -1;
  }
  while (!exit_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
  }
  std::cout << "wait to quit" << std::endl;
  media_producer->Stop();
  media_producer->DeInit();

  return 0;
}
