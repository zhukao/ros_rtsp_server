#ifndef RTSP_SERVER_H_
#define RTSP_SERVER_H_

#include <memory>
#include <thread>
#include <fstream>
#include <string>
#include <time.h>

class RTSPServer;
class TaskScheduler;
class UsageEnvironment;
class ServerMediaSession;

namespace rtspcomponent {

enum class VideoType { H264 = 0, H265 };

struct RtspServerConfig {
  // url : "rtsp://[ip_]:[port_]/[stream_name]",
  // e.g.: "rtsp://127.0.0.1:555/chn0",
  std::string stream_name{"chn0"};
  int port_ = 555;
  std::string ip_{"127.0.0.1"};
  VideoType video_type_;

  uint8_t auth_mode_ = 0;  // 0 for none
  std::string user_;
  std::string password_;
};

class RtspServer {
 public:
  explicit RtspServer(std::shared_ptr<RtspServerConfig> sp_config);
  ~RtspServer();
  /**
   * @brief load rtsp config
   * @return 0: ok, -1: fail
   */
  int Init();

  /**
   * @brief create RTSPServerRun thread
   * @return 0: ok, -1: fail
   */
  int Start();

  /**
   * @brief stop thread
   * @return 0: ok, -1: fail
   */  
  int Stop();

   /**
   * @brief distory resource
   * @return 0: ok, -1: fail
   */ 
  int DeInit();

  /**
   * @brief send data to rtsp frame source
   * @param in buf: frame
   * @param in buf_len: frame size
   * @param in media_type: H264/H265
   * @return 0: ok, -1: fail
   */
  int SendData(const unsigned char* buf, int buf_len,
        int media_type);

 private:
   /**
   * @brief create rtsp server and add mediasession
   * @return 0: ok, -1: fail
   */ 
  int RTSPServerRun();   // server 运行在独立线程

 private:
  std::shared_ptr<RtspServerConfig> sp_config_ = nullptr;
  RTSPServer *rtsp_server_;   // live555 rtsp server
  TaskScheduler *scheduler_;
  UsageEnvironment *env_;
  ServerMediaSession *server_media_session_;
  std::shared_ptr<std::thread> rtsp_server_thread_;
  volatile char watch_variable_;
};
}  // namespace RTSPComponent
#endif
