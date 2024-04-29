
#ifndef MEDIA_PRODUCER_H_
#define MEDIA_PRODUCER_H_

#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#ifdef __cplusplus
extern "C" {
#endif
#include "libswresample/swresample.h"
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/avutil.h"
#include "libavutil/imgutils.h"
#include "libavutil/hwcontext.h"
#ifdef __cplusplus
}
#endif

#include "rtsp_server/rtsp_server.h"

using rtspcomponent::RtspServerConfig;
using rtspcomponent::VideoType;
using rtspcomponent::RtspServer;

class MediaProducer {
 public:
  explicit MediaProducer(std::shared_ptr<RtspServerConfig> sp_rtsp_config, std::string video_path);
  ~MediaProducer();
  int Init();
  int DeInit();
  int Start();
  int Stop();

 private:
  int GetMediaThread();

 private:
  std::shared_ptr<RtspServer> rtsp_server_ = nullptr;
  std::shared_ptr<RtspServerConfig> sp_rtsp_config_ = nullptr;
  AVFormatContext * input_ctx_ = nullptr;

  bool thread_exit_flag_;
  std::string media_source_list_;
  std::shared_ptr<std::thread> get_frame_thread_ = nullptr;
  int video_type_;
};

MediaProducer::MediaProducer(std::shared_ptr<RtspServerConfig> sp_rtsp_config, std::string video_path) {
  if (!sp_rtsp_config) {
    throw std::runtime_error("rtsp config is null");
  }
  sp_rtsp_config_ = std::make_shared<RtspServerConfig>();
  *sp_rtsp_config_ = *sp_rtsp_config;
  media_source_list_ = video_path;

  rtsp_server_ = std::make_shared<RtspServer>(sp_rtsp_config_);
}

MediaProducer::~MediaProducer() {}

int MediaProducer::Init() {
  auto ret = rtsp_server_->Init();
  if (ret != 0) {
    std::cerr << "rtspserver init failed" << std::endl;
    return -1;
  }

  return 0;
}

int MediaProducer::DeInit() {
  return 0;
}

int MediaProducer::Start() {
  // 启动rtsp_server_
  auto ret = rtsp_server_->Start();
  if (ret != 0) {
    std::cerr << "rtsp server start failed" << std::endl;
    return -1;
  }

  // 创建线程读取媒体文件并推送
  {
    thread_exit_flag_ = false;
    input_ctx_ = nullptr;
    get_frame_thread_ =
      std::make_shared<std::thread>(&MediaProducer::GetMediaThread, this);
  }

  return 0;
}

int MediaProducer::Stop() {
  if (get_frame_thread_ && get_frame_thread_->joinable()) {
    thread_exit_flag_ = true;
    get_frame_thread_->join();
    get_frame_thread_ = nullptr;
  }

  if (rtsp_server_) {
    rtsp_server_->Stop();
  }
  return 0;
}

int MediaProducer::GetMediaThread() {
  while (!thread_exit_flag_) {
    AVPacket packet;
    auto ret = avformat_open_input(&input_ctx_,
              media_source_list_.c_str(), NULL, NULL);
    if (ret != 0) {
      std::cerr << "open media file failed" << std::endl;
      return -1;
    }
    ret = av_find_best_stream(input_ctx_,
            AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
    if (ret < 0) {
        std::cerr << "av_find_best_stream failed" << std::endl;
        return -1;
    }

    while (!thread_exit_flag_) {
      ret = av_read_frame(input_ctx_, &packet);
      if (ret < 0) {
        av_packet_unref(&packet);
        break;
      } else {
        // 向通道chn_id 转发数据
        rtsp_server_->SendData(packet.data, packet.size,
                        static_cast<int>(sp_rtsp_config_->video_type_));

      #if 0
        std::string file_name = "dump_stream.264";
        std::fstream fout(file_name, std::ios::out |
                          std::ios::binary | std::ios::app);
        fout.write((const char *)packet.data, packet.size);
         fout.close();
      #endif
      }
        av_packet_unref(&packet);
        usleep(40000);
      }
      avformat_close_input(&input_ctx_);
  }
  return 0;
}

#endif