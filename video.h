#ifndef VIDEO_DAEMON_VIDEO_H
#define VIDEO_DAEMON_VIDEO_H

#include <rk_common.h>

typedef struct {
    RK_S32 s32FirstFrameStartQp;
    RK_U32 u32StepQp;
    RK_U32 u32MinQp;
    RK_U32 u32MaxQp;
    RK_U32 u32MinIQp;
    RK_U32 u32MaxIQp;
    RK_S32 s32DeltIpQp;
    RK_S32 s32MaxReEncodeTimes;
    RK_U32 u32FrmMaxQp;
    RK_U32 u32FrmMinQp;
    RK_U32 u32FrmMinIQp;
    RK_U32 u32FrmMaxIQp;
    RK_U32 u32MotionStaticSwitchFrmQp;
} RcQpParams;

typedef struct {
    RcQpParams h264;
    RcQpParams h265;
} VideoRcConfig;

int video_init();
void video_shutdown();
void *run_detect_format(void *arg);
void video_start_streaming();
void video_stop_streaming();

void video_set_quality_factor(float factor);
void video_set_encodec_type(RK_CODEC_ID_E type);
void video_set_yolo_enable(int enable);
void video_set_rc(const VideoRcConfig *rc);
void video_get_rc(VideoRcConfig *rc);

int udp_socket_init(int port);
int udp_socket_send(const u_int8_t* frame, ssize_t len);

#endif //VIDEO_DAEMON_VIDEO_H
