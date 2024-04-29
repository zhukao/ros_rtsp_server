#include <iostream>
#include <BasicUsageEnvironment.hh>
#include <InputFile.hh>
#include <RTSPServerSupportingHTTPStreaming.hh>
#include <liveMedia.hh>
#include "OnDemandServerMediaSubsession.hh"
#include <fcntl.h>

#include "rtsp_server/rtsp_server.h"
#include "rtsp_component.h"

namespace rtspcomponent {
RtspServer::RtspServer(std::shared_ptr<RtspServerConfig> sp_config) {
  sp_config_ = std::make_shared<RtspServerConfig>();
  if (sp_config) {
    *sp_config_ = *sp_config;
  }
}

RtspServer::~RtspServer() {
}

int RtspServer::Init() {
  rtsp_server_thread_ = nullptr;
  watch_variable_ = 0;
  return 0;
}

int RtspServer::Start() {
  if (nullptr == rtsp_server_thread_) {
    rtsp_server_thread_ =
      std::make_shared<std::thread>(&RtspServer::RTSPServerRun, this);
  }

  return 0;
}

int RtspServer::Stop() {
  if (rtsp_server_thread_ && rtsp_server_thread_->joinable()) {
    watch_variable_ = 1;
    rtsp_server_thread_->join();
    rtsp_server_thread_ = nullptr;
  }
  return 0;
}

int RtspServer::DeInit() {
  return 0;
}

int RtspServer::SendData(const unsigned char* buf, int buf_len,
               int media_type) {
  if (media_type == static_cast<int>(VideoType::H264) || media_type == static_cast<int>(VideoType::H265)) {
    if (buf_len >= 4 && buf[0] == 0 && buf[1] == 0
        && buf[2] == 0 && buf[3] == 1) {
      H264Or5FramedSource::OnH264Or5Frame(
        buf+4, buf_len-4, sp_config_->stream_name.c_str());
    } else if (buf_len >= 4 && buf[0] == 0 && buf[1] == 0 && buf[2] == 1) {
      H264Or5FramedSource::OnH264Or5Frame(
        buf+3, buf_len-3, sp_config_->stream_name.c_str());
    }
  }
  return 0;
}

int RtspServer::RTSPServerRun() {
  scheduler_ = BasicTaskScheduler::createNew();
  env_ = BasicUsageEnvironment::createNew(*scheduler_);

  UserAuthenticationDatabase *authDB = NULL;

  if (sp_config_->auth_mode_) {
    authDB = new UserAuthenticationDatabase;
    authDB->addUserRecord(sp_config_->user_.c_str(), sp_config_->password_.c_str());
  }

  *env_ << "RTSP server use auth: " << sp_config_->auth_mode_
       << "  username: " << sp_config_->user_.c_str() << " password "
       << sp_config_->password_.c_str() << "\n";
  OutPacketBuffer::maxSize = 2000000;

  portNumBits rtspServerPortNum = sp_config_->port_;
  rtsp_server_ = RTSPServer::createNew(*env_, rtspServerPortNum, authDB);
  if (rtsp_server_ == NULL) {
    *env_ << "Failed to create RTSP Server: " << env_->getResultMsg() << "\n";
    exit(1);
  }

  std::cout << " rtsp url = " << sp_config_->stream_name << std::endl;

  ServerMediaSession *sms = ServerMediaSession::createNew(
    *env_, sp_config_->stream_name.c_str(), "rtsp server", "session");

  if (sp_config_->video_type_ == VideoType::H264) {
    sms->addSubsession(H264ServerMediaSubsession::createNew(*env_,
      True));
  } else if (sp_config_->video_type_ == VideoType::H265) {
    sms->addSubsession(H265ServerMediaSubsession::createNew(*env_,
      True));
  }

  rtsp_server_->addServerMediaSession(sms);

  char *url = rtsp_server_->rtspURL(sms);
  *env_ << "Play this stream using the URL \n\t" << url << "\n";
  delete[] url;

  env_->taskScheduler().doEventLoop(&watch_variable_);
  std::cout << "reclaim and delete scheduler..." << std::endl;
  Medium::close(rtsp_server_);
  env_->reclaim();
  delete scheduler_;
  rtsp_server_ = NULL;
  env_ = NULL;
  scheduler_ = NULL;
  return 0;
}
}  // namespace rtspcomponent
