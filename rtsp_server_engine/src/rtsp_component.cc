#include "rtsp_component.h"
#include <BasicUsageEnvironment.hh>
#include <InputFile.hh>
#include <RTSPServerSupportingHTTPStreaming.hh>
#include <liveMedia.hh>

#include "OnDemandServerMediaSubsession.hh"
#include <iostream>

pthread_mutex_t gs_mutex;

H264Or5FramedSource *H264Or5FramedSource::createNew(
  UsageEnvironment &env, const char* streamName) {
  H264Or5FramedSource *source = new H264Or5FramedSource(env);

  memcpy(source->stream_name_, streamName, strlen(streamName));
  return source;
}

H264Or5FramedSource *H264Or5FramedSource::frame_sources_[100];

void H264Or5FramedSource::OnH264Or5Frame(
  const unsigned char *buff, int len, const char* streamName) {
  int count = sizeof(frame_sources_) / sizeof(H264Or5FramedSource *);
  for (int i = 0; i < count; i++) {
    pthread_mutex_lock(&gs_mutex);
    H264Or5FramedSource *source = frame_sources_[i];
    if (source != NULL && !strcmp(source->GetStreamName(), streamName)) {
        source->GetH264Or5Frame(buff, len);
    }

    pthread_mutex_unlock(&gs_mutex);
  }
}

unsigned int H264Or5FramedSource::referenceCount = 0;

H264Or5FramedSource::H264Or5FramedSource(UsageEnvironment &env)
  : FramedSource(env) {
  envir() << "H264Or5FramedSource create . " << this << "\n";
  frame_buffer_ = new unsigned char[MAX_FRAME_SIZE]();
  video_buffer_ = new unsigned char[MAX_VIDEO_SIZE]();
  video_front_ = 0;
  video_tail_ = 0;
  video_current_cnt_ = 0;
  video_buff_cnt_ = 0;
  memset(video_offset_, 0, sizeof(video_offset_));

  referenceCount++;

  eventTriggerId = 0;
  if (eventTriggerId == 0) {
    eventTriggerId = envir().taskScheduler().createEventTrigger(SendFrame);
  }

  AddSource(this);
}

H264Or5FramedSource::~H264Or5FramedSource() {
  envir() << "H264Or5FramedSource destory." << this << " \n";
  if (frame_buffer_) {
    delete []frame_buffer_;
  }
  if (video_buffer_) {
    delete []video_buffer_;
  }
  EraseSource(this);
  referenceCount--;

  if (eventTriggerId > 0) {
    envir().taskScheduler().deleteEventTrigger(eventTriggerId);
    eventTriggerId = 0;
  }
}

int H264Or5FramedSource::AddSource(H264Or5FramedSource *source) {
  int count = sizeof(frame_sources_) / sizeof(H264Or5FramedSource *);
  for (int i = 0; i < count; i++) {
    if (frame_sources_[i] == NULL) {
      envir() << "H264Or5FramedSource " << source
              << " addSource i = " << i << "\n";
      frame_sources_[i] = source;
      return i;
    }
  }
  envir() << "H264Or5FramedSource::addSource failed\n ";
  return -1;
}

int H264Or5FramedSource::EraseSource(
  H264Or5FramedSource *source) {
  pthread_mutex_lock(&gs_mutex);
  int count = sizeof(frame_sources_) / sizeof(H264Or5FramedSource *);
  for (int i = 0; i < count; i++) {
    if (frame_sources_[i] == source) {
      envir() << "H264Or5FramedSource " << source
              << " eraseSource i = " << i << "\n";
      frame_sources_[i] = NULL;
      pthread_mutex_unlock(&gs_mutex);
      return i;
    }
  }
  pthread_mutex_unlock(&gs_mutex);
  return -1;
}

int H264Or5FramedSource::SetStreamName(char* stream_name) {
  memcpy(stream_name_, stream_name, strlen(stream_name));
  std::cout << "set stream name = " << stream_name_ << "\n";
  return 0;
}

char* H264Or5FramedSource::GetStreamName() {
  return stream_name_;
}

void H264Or5FramedSource::GetH264Or5Frame(const unsigned char *buff, int len) {
  int offset = 0;
  memcpy(frame_buffer_ + offset, buff, len);
  offset += len;

  int total_cnt = sizeof(video_offset_) / sizeof(int) - 1;

  //缓存满需要另外处理
  if (video_tail_ + offset > MAX_VIDEO_SIZE || video_buff_cnt_ >= total_cnt) {
    envir() << "H264Or5FramedSource drop frame video_current_cnt_:"
            << video_current_cnt_ << " video_buff_cnt_:" << video_buff_cnt_
            << " video_tail_:" << video_tail_ << " video_front_:"
            << video_front_ << " object:" << this << "\n";

    video_front_ = 0;
    video_tail_ = 0;
    video_current_cnt_ = 0;
    video_buff_cnt_ = 0;
    memset(video_offset_, 0, sizeof(video_offset_));

    return;
  }

  memcpy(video_buffer_ + video_tail_, frame_buffer_, offset);
  video_tail_ += offset;
  video_offset_[video_buff_cnt_] = video_tail_;
  video_buff_cnt_++;

  if (eventTriggerId > 0) {
    envir().taskScheduler().triggerEvent(eventTriggerId, this);
  }
}

void H264Or5FramedSource::SendFrame(void *client) {
  H264Or5FramedSource *source = (H264Or5FramedSource *) client;
  source->doGetNextFrame();
}

void H264Or5FramedSource::doGetNextFrame() {
  if (isCurrentlyAwaitingData() == False) { //上次读的数据还没有处理完
    return;
  }

  if (video_tail_ > video_front_ && eventTriggerId > 0) {
    pthread_mutex_lock(&gs_mutex);

    int offset = video_offset_[video_current_cnt_];
    int size = offset - video_front_;

    memcpy(fTo, video_buffer_ + video_front_, size);

    fFrameSize = size;
    video_front_ = offset;
    video_current_cnt_++;

    if (video_front_ >= video_tail_) {
      video_front_ = 0;
      video_tail_ = 0;
      video_current_cnt_ = 0;
      video_buff_cnt_ = 0;
      memset(video_offset_, 0, sizeof(video_offset_));
    }

    gettimeofday(&fPresentationTime, NULL);
    afterGetting(this);

    pthread_mutex_unlock(&gs_mutex);
    return;
  }
}

void H264Or5FramedSource::doStopGettingFrames() {
  envir() << "H264Or5FramedSource::doStopGettingFrames\n";
  EraseSource(this);

  pthread_mutex_lock(&gs_mutex);

  if (eventTriggerId > 0) {
    envir().taskScheduler().deleteEventTrigger(eventTriggerId);
    eventTriggerId = 0;
  }

  video_front_ = 0;
  video_tail_ = 0;
  video_current_cnt_ = 0;
  video_buff_cnt_ = 0;
  memset(video_offset_, 0, sizeof(video_offset_));

  pthread_mutex_unlock(&gs_mutex);

  FramedSource::doStopGettingFrames();
}

H264ServerMediaSubsession *H264ServerMediaSubsession::createNew(
    UsageEnvironment &env, bool reuseFirstSource) {
  H264ServerMediaSubsession *subsession =
      new H264ServerMediaSubsession(env, reuseFirstSource);
  return subsession;
}

H264ServerMediaSubsession::H264ServerMediaSubsession(
    UsageEnvironment &env, bool reuseFirstSource)
    : OnDemandServerMediaSubsession(env, reuseFirstSource) {}

H264ServerMediaSubsession::~H264ServerMediaSubsession() {}

FramedSource *H264ServerMediaSubsession::createNewStreamSource(
    unsigned clientSessionId, unsigned &estBitrate) {
  envir() << "createNewStreamSource H264 clientSessionId:" << clientSessionId
          << "\n";
  estBitrate = 8000;  // 8000kbit/s
  FramedSource *source = H264Or5FramedSource::createNew(
    envir(), fParentSession->streamName());

  // add a framer in front of the source:
  source = H264VideoStreamDiscreteFramer::createNew(envir(), source);
  return source;
}

RTPSink *H264ServerMediaSubsession::createNewRTPSink(
    Groupsock *rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic,
    FramedSource *inputSource) {
  envir() << "createNewRTPSink H264 " << "\n";
  return H264VideoRTPSink::createNew(envir(), rtpGroupsock,
                                     rtpPayloadTypeIfDynamic);
}

H265ServerMediaSubsession *H265ServerMediaSubsession::createNew(
    UsageEnvironment &env, bool reuseFirstSource) {
  H265ServerMediaSubsession *subsession =
      new H265ServerMediaSubsession(env, reuseFirstSource);
  return subsession;
}

H265ServerMediaSubsession::H265ServerMediaSubsession(
    UsageEnvironment &env, bool reuseFirstSource)
    : OnDemandServerMediaSubsession(env, reuseFirstSource) {}

H265ServerMediaSubsession::~H265ServerMediaSubsession() {}

FramedSource *H265ServerMediaSubsession::createNewStreamSource(
    unsigned clientSessionId, unsigned &estBitrate) {
  envir() << "createNewStreamSource H265 clientSessionId:" << clientSessionId
          << "\n";
  estBitrate = 8000;  // 8000kbit/s
  FramedSource *source = H264Or5FramedSource::createNew(
    envir(), fParentSession->streamName());

  // add a framer in front of the source:
  source = H265VideoStreamDiscreteFramer::createNew(envir(), source);
  return source;
}

RTPSink *H265ServerMediaSubsession::createNewRTPSink(
    Groupsock *rtpGroupsock, unsigned char rtpPayloadTypeIfDynamic,
    FramedSource *inputSource) {
  envir() << "createNewRTPSink H265 " << "\n";
  return H265VideoRTPSink::createNew(envir(), rtpGroupsock,
                                     rtpPayloadTypeIfDynamic);
}
