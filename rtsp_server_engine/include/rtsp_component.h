#ifndef RTSP_COMPONENT_H
#define RTSP_COMPONENT_H

#include <liveMedia.hh>
#include <thread>

#define MAX_FRAME_SIZE 300000
#define MAX_VIDEO_SIZE 1000000

class H264Or5FramedSource : public FramedSource {
 public:
  static H264Or5FramedSource *createNew(UsageEnvironment &env,
            const char* stream_name);
  static void OnH264Or5Frame(const unsigned char *buff, int len, const char* chn_id);

  static void SendFrame(void *client);

  static H264Or5FramedSource *frame_sources_[100];

  int SetStreamName(char* streamName);
  char* GetStreamName();

 public:
  EventTriggerId eventTriggerId;

  // used to count how many instances of this class currently exist
  static unsigned int referenceCount;

 protected:
  explicit H264Or5FramedSource(UsageEnvironment &env);
  ~H264Or5FramedSource();

 private:
  virtual void doGetNextFrame();
  virtual void doStopGettingFrames();

  void GetH264Or5Frame(const unsigned char *buff, int len);

  int AddSource(H264Or5FramedSource *source);
  int EraseSource(H264Or5FramedSource *source);


  unsigned char *frame_buffer_;
  unsigned char *video_buffer_;

  int video_buff_cnt_;
  int video_current_cnt_;
  // video buffer 头指针，读取发送以这个开始
  int video_front_;
  // video buffer 尾指针,缓存数据以这个结束
  int video_tail_;
  int video_offset_[1000];

  char stream_name_[128];
};

class H264ServerMediaSubsession : public OnDemandServerMediaSubsession {
 public:
  static H264ServerMediaSubsession *
  createNew(UsageEnvironment &env, bool reuseFirstSource);

 protected:
  H264ServerMediaSubsession(UsageEnvironment &env,
                                bool reuseFirstSource);
  ~H264ServerMediaSubsession();

  virtual FramedSource *createNewStreamSource(unsigned clientSessionId,
                                              unsigned &estBitrate);
  // "estBitrate" is the stream's estimated bitrate, in kbps
  virtual RTPSink *createNewRTPSink(Groupsock *rtpGroupsock,
                                    unsigned char rtpPayloadTypeIfDynamic,
                                    FramedSource *inputSource);

 private:
};

class H265ServerMediaSubsession : public OnDemandServerMediaSubsession {
 public:
  static H265ServerMediaSubsession *
  createNew(UsageEnvironment &env, bool reuseFirstSource);

 protected:
  H265ServerMediaSubsession(UsageEnvironment &env,
                                bool reuseFirstSource);
  ~H265ServerMediaSubsession();

  virtual FramedSource *createNewStreamSource(unsigned clientSessionId,
                                              unsigned &estBitrate);
  // "estBitrate" is the stream's estimated bitrate, in kbps
  virtual RTPSink *createNewRTPSink(Groupsock *rtpGroupsock,
                                    unsigned char rtpPayloadTypeIfDynamic,
                                    FramedSource *inputSource);

 private:
};
#endif
