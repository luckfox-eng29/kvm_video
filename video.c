#define _POSIX_C_SOURCE 200809L
#include <unistd.h>
#include <time.h>
#include <rk_type.h>
#include <rk_mpi_venc.h>
#include <string.h>
#include <rk_debug.h>
#include <malloc.h>
#include <stdbool.h>
#include <rk_mpi_mb.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <stdatomic.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <rk_mpi_mmz.h>
#include <pthread.h>
#include <assert.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include "video.h"
#include "ctrl.h"
#include "yolo_c.h"
#include "rk_mpi_rgn.h"
#include "osd/overlay.h"
#include "preprocess.h"

#define VIDEO_DEV "/dev/video0"
#define SUB_DEV "/dev/v4l-subdev2"
#define HDMI_STATUS_FILE "/sys/kernel/debug/4-0050/hdmirx/status"

#define RK_ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))
#define RK_ALIGN_2(x) RK_ALIGN(x, 2)
#define RK_ALIGN_16(x) RK_ALIGN(x, 16)
#define RK_ALIGN_32(x) RK_ALIGN(x, 32)

#define COMBO_JPEG_CHN 1

// A failed RK_MPI_VENC_GetStream() retries without sleeping would spin the single
// Cortex-A7 core at 100% and starve the whole system.
#define JPEG_GET_STREAM_MAX_FAILS 5

// How long the snapshot thread waits for the encoder before telling the control
// client that the pending request is stuck. Without this the client can only
// report its own timeout, which says nothing about the actual cause.
#define JPEG_VENC_WAIT_MAX_MS 1000

int sub_dev_fd = -1;
#define VENC_CHANNEL 0
MB_POOL memPool = MB_INVALID_POOLID;

static pthread_t *jpeg_thread = NULL;
static volatile bool jpeg_running = false;
static volatile bool jpeg_initialized = false;
static JpegMode jpeg_mode = JPEG_SINGLE_SHOT;
static pthread_mutex_t jpeg_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t jpeg_cond = PTHREAD_COND_INITIALIZER;
static bool jpeg_trigger = false;
static RK_U32 jpeg_width = 0;
static RK_U32 jpeg_height = 0;

bool should_exit = false;
float quality_factor = 1.0f;
RK_CODEC_ID_E encodec_type = RK_VIDEO_ID_AVC;

uint32_t detected_width, detected_height;
bool detected_signal = false, streaming_flag = false;
static pthread_mutex_t streaming_mutex = PTHREAD_MUTEX_INITIALIZER;

static VideoRcConfig video_rc_config = {
    .h264 = {
        .s32FirstFrameStartQp = 50,
        .u32StepQp = 48,
        .u32MinQp = 48,
        .u32MaxQp = 51,
        .u32MinIQp = 48,
        .u32MaxIQp = 51,
        .s32DeltIpQp = 7,
        .s32MaxReEncodeTimes = 2,
        .u32FrmMaxQp = 51,
        .u32FrmMinQp = 48,
        .u32FrmMinIQp = 51,
        .u32FrmMaxIQp = 48,
        .u32MotionStaticSwitchFrmQp = 50,
    },
    .h265 = {
        .s32FirstFrameStartQp = 50,
        .u32StepQp = 48,
        .u32MinQp = 48,
        .u32MaxQp = 51,
        .u32MinIQp = 48,
        .u32MaxIQp = 51,
        .s32DeltIpQp = 7,
        .s32MaxReEncodeTimes = 2,
        .u32FrmMaxQp = 51,
        .u32FrmMinQp = 48,
        .u32FrmMinIQp = 51,
        .u32FrmMaxIQp = 48,
        .u32MotionStaticSwitchFrmQp = 50,
    },
};

static void *venc_read_stream(void *arg);
static void* yolo_infer_thread(void* arg);
static void *run_preview_socket_thread(void *arg);
static int32_t venc_start(RK_CODEC_ID_E enType, int32_t bitrate, int32_t max_bitrate, int32_t width, int32_t height, const VideoRcConfig *rc);
static int32_t combo_jpeg_init(RK_U32 width, RK_U32 height);
static int32_t combo_jpeg_stop(void);
static void *combo_jpeg_get_stream(void *arg);

static pthread_t* yolo_thread = NULL;
static volatile bool yolo_running = false;
static pthread_mutex_t yolo_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t yolo_cond = PTHREAD_COND_INITIALIZER;
static uint8_t* yolo_yuyv = NULL;
static size_t yolo_yuyv_size = 0;
static uint8_t* yolo_rgb = NULL;
static size_t yolo_rgb_size = 0;
static uint32_t yolo_width = 0, yolo_height = 0;
static bool yolo_has_frame = false;
static uint8_t* yolo_model_buf = NULL;
static size_t yolo_model_buf_size = 0;
static int yolo_model_h = 0, yolo_model_w = 0, yolo_model_c = 0;
static int yolo_fd = -1;
static void* yolo_va = NULL;
static int yolo_bound_fd = -1;

static const int YOLO_MAX_DETS = 128;

static bool check_hdmi_connection_from_debugfs()
{
    FILE *fp = fopen(HDMI_STATUS_FILE, "r");
    if (fp == NULL) {
        RK_LOGE("Failed to open HDMI status file: %s", HDMI_STATUS_FILE);
        return false;
    }

    char line[512];
    char status[32] = {0};

    while (fgets(line, sizeof(line), fp) != NULL) {
        if (strncmp(line, "status:", 7) == 0) {
            sscanf(line, "status: %s", status);
            break;
        }
    }

    fclose(fp);

    RK_LOGI("HDMI status: %s", status);
    return strcmp(status, "plugin") == 0;
}

RK_U64 get_us()
{
    struct timespec time = {0, 0};
    clock_gettime(CLOCK_MONOTONIC, &time);
    return (RK_U64)time.tv_sec * 1000000 + (RK_U64)time.tv_nsec / 1000; /* microseconds */
}

double calculate_bitrate(float bitrate_factor, int width, int height)
{
    const int32_t base_bitrate_high = 2000;
    const int32_t base_bitrate_low = 512;

    double pixels = (double)width * height;
    double ref_pixels = 1920.0 * 1080.0;

    double scale_factor = pixels / ref_pixels;

    int32_t base_bitrate = base_bitrate_low + (int32_t)((base_bitrate_high - base_bitrate_low) * bitrate_factor);

    int32_t bitrate = (int32_t)(base_bitrate * scale_factor);

    const int32_t min_bitrate = 100;
    if (bitrate < min_bitrate)
    {
        bitrate = min_bitrate;
    }

    return bitrate;
}

static void populate_rc_param(VENC_RC_PARAM_S *stRcParam, RK_CODEC_ID_E enType, const VideoRcConfig *rc)
{
    memset(stRcParam, 0, sizeof(VENC_RC_PARAM_S));
    stRcParam->s32FirstFrameStartQp = rc->h264.s32FirstFrameStartQp;

    const RcQpParams *qp = (enType == RK_VIDEO_ID_HEVC) ? &rc->h265 : &rc->h264;
    stRcParam->stParamH264.u32StepQp = qp->u32StepQp;
    stRcParam->stParamH264.u32MinQp = qp->u32MinQp;
    stRcParam->stParamH264.u32MaxQp = qp->u32MaxQp;
    stRcParam->stParamH264.u32MinIQp = qp->u32MinIQp;
    stRcParam->stParamH264.u32MaxIQp = qp->u32MaxIQp;
    stRcParam->stParamH264.s32DeltIpQp = qp->s32DeltIpQp;
    stRcParam->stParamH264.s32MaxReEncodeTimes = qp->s32MaxReEncodeTimes;
    stRcParam->stParamH264.u32FrmMaxQp = qp->u32FrmMaxQp;
    stRcParam->stParamH264.u32FrmMinQp = qp->u32FrmMinQp;
    stRcParam->stParamH264.u32FrmMinIQp = qp->u32FrmMinIQp;
    stRcParam->stParamH264.u32FrmMaxIQp = qp->u32FrmMaxIQp;
    stRcParam->stParamH264.u32MotionStaticSwitchFrmQp = qp->u32MotionStaticSwitchFrmQp;

    stRcParam->stParamH265.u32StepQp = qp->u32StepQp;
    stRcParam->stParamH265.u32MinQp = qp->u32MinQp;
    stRcParam->stParamH265.u32MaxQp = qp->u32MaxQp;
    stRcParam->stParamH265.u32MinIQp = qp->u32MinIQp;
    stRcParam->stParamH265.u32MaxIQp = qp->u32MaxIQp;
    stRcParam->stParamH265.s32DeltIpQp = qp->s32DeltIpQp;
    stRcParam->stParamH265.s32MaxReEncodeTimes = qp->s32MaxReEncodeTimes;
    stRcParam->stParamH265.u32FrmMaxQp = qp->u32FrmMaxQp;
    stRcParam->stParamH265.u32FrmMinQp = qp->u32FrmMinQp;
    stRcParam->stParamH265.u32FrmMinIQp = qp->u32FrmMinIQp;
    stRcParam->stParamH265.u32FrmMaxIQp = qp->u32FrmMaxIQp;
    stRcParam->stParamH265.u32MotionStaticSwitchFrmQp = qp->u32MotionStaticSwitchFrmQp;
}

static void populate_venc_attr(VENC_CHN_ATTR_S *stAttr, RK_CODEC_ID_E enType, RK_U32 bitrate, RK_U32 max_bitrate, RK_U32 width, RK_U32 height)
{
    memset(stAttr, 0, sizeof(VENC_CHN_ATTR_S));


    if (enType == RK_VIDEO_ID_HEVC)
    {
        stAttr->stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        stAttr->stRcAttr.stH265Vbr.u32BitRate = bitrate;
        stAttr->stRcAttr.stH265Vbr.u32MaxBitRate = max_bitrate;
        stAttr->stRcAttr.stH265Vbr.u32Gop = 60;
    }
    else if (enType == RK_VIDEO_ID_AVC)
    {
        stAttr->stRcAttr.enRcMode = VENC_RC_MODE_H264VBR;
        stAttr->stRcAttr.stH264Vbr.u32BitRate = bitrate;
        stAttr->stRcAttr.stH264Vbr.u32MaxBitRate = max_bitrate;
        stAttr->stRcAttr.stH264Vbr.u32Gop = 60;
        stAttr->stVencAttr.u32Profile = H264E_PROFILE_MAIN;
    }
 
    stAttr->stVencAttr.enType = enType;
    stAttr->stVencAttr.enPixelFormat = RK_FMT_YUV422_YUYV;
    stAttr->stVencAttr.u32PicWidth = width;
    stAttr->stVencAttr.u32PicHeight = height;
    // stAttr->stVencAttr.u32VirWidth = (width + 15) & (~15);
    // stAttr->stVencAttr.u32VirHeight = (height + 15) & (~15);
    stAttr->stVencAttr.u32VirWidth = RK_ALIGN_2(width);
    stAttr->stVencAttr.u32VirHeight = RK_ALIGN_2(height);
    stAttr->stVencAttr.u32StreamBufCnt = 3;
    stAttr->stVencAttr.u32BufSize = width * height * 3 / 2;
    stAttr->stVencAttr.enMirror = MIRROR_NONE;
}

pthread_t *venc_read_thread = NULL;
volatile bool venc_running = false;
static int32_t venc_start(RK_CODEC_ID_E enType, int32_t bitrate, int32_t max_bitrate, int32_t width, int32_t height, const VideoRcConfig *rc)
{
    int32_t ret;
    VENC_CHN_ATTR_S stAttr;
    populate_venc_attr(&stAttr, enType, bitrate, max_bitrate, width, height);
    VENC_RC_PARAM_S stRcParam;
    populate_rc_param(&stRcParam, enType, rc);

    ret = RK_MPI_VENC_CreateChn(VENC_CHANNEL, &stAttr);
    if (ret < 0)
    {
        RK_LOGE("error RK_MPI_VENC_CreateChn, %d", ret);
        return ret;
    }


    ret = RK_MPI_VENC_GetRcParam(VENC_CHANNEL, &stRcParam);
    if (ret < 0)
    {
        RK_LOGE("error RK_MPI_VENC_GetRcParam, %d", ret);
    }
    else
    {
        printf("===== VENC RC PARAM (GET) =====\n");
        printf("s32FirstFrameStartQp: %d\n", stRcParam.s32FirstFrameStartQp);
        printf("stParamH264.u32StepQp: %u\n", stRcParam.stParamH264.u32StepQp);
        printf("stParamH264.u32MinQp: %u\n", stRcParam.stParamH264.u32MinQp);
        printf("stParamH264.u32MaxQp: %u\n", stRcParam.stParamH264.u32MaxQp);
        printf("stParamH264.u32MinIQp: %u\n", stRcParam.stParamH264.u32MinIQp);
        printf("stParamH264.u32MaxIQp: %u\n", stRcParam.stParamH264.u32MaxIQp);
        printf("stParamH264.s32DeltIpQp: %d\n", stRcParam.stParamH264.s32DeltIpQp);
        printf("stParamH264.s32MaxReEncodeTimes: %d\n", stRcParam.stParamH264.s32MaxReEncodeTimes);
        printf("stParamH264.u32FrmMaxQp: %u\n", stRcParam.stParamH264.u32FrmMaxQp);
        printf("stParamH264.u32FrmMinQp: %u\n", stRcParam.stParamH264.u32FrmMinQp);
        printf("stParamH264.u32FrmMinIQp: %u\n", stRcParam.stParamH264.u32FrmMinIQp);
        printf("stParamH264.u32FrmMaxIQp: %u\n", stRcParam.stParamH264.u32FrmMaxIQp);
        printf("stParamH264.u32MotionStaticSwitchFrmQp: %u\n", stRcParam.stParamH264.u32MotionStaticSwitchFrmQp);

        printf("stParamH265.u32StepQp: %u\n", stRcParam.stParamH265.u32StepQp);
        printf("stParamH265.u32MinQp: %u\n", stRcParam.stParamH265.u32MinQp);
        printf("stParamH265.u32MaxQp: %u\n", stRcParam.stParamH265.u32MaxQp);
        printf("stParamH265.u32MinIQp: %u\n", stRcParam.stParamH265.u32MinIQp);
        printf("stParamH265.u32MaxIQp: %u\n", stRcParam.stParamH265.u32MaxIQp);
        printf("stParamH265.s32DeltIpQp: %d\n", stRcParam.stParamH265.s32DeltIpQp);
        printf("stParamH265.s32MaxReEncodeTimes: %d\n", stRcParam.stParamH265.s32MaxReEncodeTimes);
        printf("stParamH265.u32FrmMaxQp: %u\n", stRcParam.stParamH265.u32FrmMaxQp);
        printf("stParamH265.u32FrmMinQp: %u\n", stRcParam.stParamH265.u32FrmMinQp);
        printf("stParamH265.u32FrmMinIQp: %u\n", stRcParam.stParamH265.u32FrmMinIQp);
        printf("stParamH265.u32FrmMaxIQp: %u\n", stRcParam.stParamH265.u32FrmMaxIQp);
        printf("stParamH265.u32MotionStaticSwitchFrmQp: %u\n", stRcParam.stParamH265.u32MotionStaticSwitchFrmQp);
        printf("=============================\n");
    }

    ret = RK_MPI_VENC_SetRcParam(VENC_CHANNEL, &stRcParam);
    if (ret < 0)
    {
        RK_LOGE("error RK_MPI_VENC_SetRcParam, %d", ret);
    }

    VENC_RECV_PIC_PARAM_S stRecvParam;
    memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
    stRecvParam.s32RecvPicNum = -1;
    ret = RK_MPI_VENC_StartRecvFrame(VENC_CHANNEL, &stRecvParam);
    if (ret < 0)
    {
        RK_LOGE("error RK_MPI_VENC_StartRecvFrame, %d", ret);
        return ret;
    }

    ret = combo_jpeg_init(width, height);
    if (ret < 0)
    {
        RK_LOGE("error combo_jpeg_init, %d", ret);
        return ret;
    }

    venc_running = true;
    venc_read_thread = malloc(sizeof(pthread_t));
    if (pthread_create(venc_read_thread, NULL, venc_read_stream, NULL) != 0)
    {
        RK_LOGE("Failed to create venc_read_thread");
        return RK_FAILURE;
    }

    return RK_SUCCESS;
}

// Join and free the snapshot thread. combo_jpeg_get_stream exits when venc_running
// goes false, so the thread is dead after every venc_stop(); without this the
// dangling non-NULL jpeg_thread pointer makes video_jpeg_get() skip creating a new
// thread and snapshots silently never complete again.
static void jpeg_thread_join(void)
{
    if (jpeg_thread != NULL)
    {
        pthread_join(*jpeg_thread, NULL);
        free(jpeg_thread);
        jpeg_thread = NULL;
    }
}

static int32_t venc_stop()
{
    venc_running = false;

    // jpeg_running/jpeg_trigger are written by video_jpeg_get() under jpeg_mutex, so
    // update them under the same lock. The lock is released before the thread join
    // below because combo_jpeg_get_stream() takes it as well.
    pthread_mutex_lock(&jpeg_mutex);
    jpeg_running = false;
    jpeg_trigger = false;
    pthread_mutex_unlock(&jpeg_mutex);

    int32_t ret = RK_SUCCESS;
    int32_t r;

    // Order matters: stop the producer, join the consumers, and only then destroy
    // channels. Destroying COMBO_JPEG_CHN while a thread is blocked in
    // RK_MPI_VENC_GetStream() hangs the encoder.
    r = RK_MPI_VENC_StopRecvFrame(VENC_CHANNEL);
    if (r != RK_SUCCESS)
    {
        RK_LOGE("Failed to stop receiving frames for VENC_CHANNEL, error code: %d", r);
        ret = r;
    }

    if (venc_read_thread != NULL)
    {
        pthread_join(*venc_read_thread, NULL);
        free(venc_read_thread);
        venc_read_thread = NULL;
    }

    jpeg_thread_join();

    if (jpeg_initialized)
    {
        combo_jpeg_stop();
    }

    r = RK_MPI_VENC_DestroyChn(VENC_CHANNEL);
    if (r != RK_SUCCESS)
    {
        RK_LOGE("Failed to destroy VENC_CHANNEL, error code: %d", r);
        ret = r;
    }

    return ret;
}

static int32_t combo_jpeg_init(RK_U32 width, RK_U32 height)
{
    int32_t ret;
    VENC_CHN_ATTR_S stAttr;
    VENC_COMBO_ATTR_S stComboAttr;
    VENC_RECV_PIC_PARAM_S stRecvParam;

    jpeg_width = width;
    jpeg_height = height;

    if (jpeg_initialized) {
        return RK_SUCCESS;
    }

    memset(&stAttr, 0, sizeof(VENC_CHN_ATTR_S));
    stAttr.stVencAttr.enType = RK_VIDEO_ID_JPEG;
    stAttr.stVencAttr.enPixelFormat = RK_FMT_YUV422_YUYV;
    stAttr.stVencAttr.u32MaxPicWidth = width;
    stAttr.stVencAttr.u32MaxPicHeight = height;
    stAttr.stVencAttr.u32PicWidth = width;
    stAttr.stVencAttr.u32PicHeight = height;
    stAttr.stVencAttr.u32VirWidth = RK_ALIGN_2(width);
    stAttr.stVencAttr.u32VirHeight = RK_ALIGN_2(height);
    stAttr.stVencAttr.u32StreamBufCnt = 3;
    stAttr.stVencAttr.u32BufSize = width * height * 3 / 2;
    stAttr.stVencAttr.stAttrJpege.bSupportDCF = RK_FALSE;
    stAttr.stVencAttr.stAttrJpege.stMPFCfg.u8LargeThumbNailNum = 0;
    stAttr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
    stAttr.stGopAttr.enGopMode = VENC_GOPMODE_INIT;

    ret = RK_MPI_VENC_CreateChn(COMBO_JPEG_CHN, &stAttr);
    if (ret != RK_SUCCESS) {
        RK_LOGE("RK_MPI_VENC_CreateChn JPEG failed: %x", ret);
        return ret;
    }

    memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
    stRecvParam.s32RecvPicNum = 1;
    ret = RK_MPI_VENC_StartRecvFrame(COMBO_JPEG_CHN, &stRecvParam);
    if (ret != RK_SUCCESS) {
        RK_LOGE("RK_MPI_VENC_StartRecvFrame JPEG failed: %x", ret);
        return ret;
    }

    memset(&stComboAttr, 0, sizeof(VENC_COMBO_ATTR_S));
    stComboAttr.bEnable = RK_TRUE;
    stComboAttr.s32ChnId = VENC_CHANNEL;
    ret = RK_MPI_VENC_SetComboAttr(COMBO_JPEG_CHN, &stComboAttr);
    if (ret != RK_SUCCESS) {
        RK_LOGE("RK_MPI_VENC_SetComboAttr failed: %x", ret);
        return ret;
    }

    jpeg_initialized = true;
    return RK_SUCCESS;
}

static int32_t combo_jpeg_stop(void)
{
    int32_t ret;
    ret = RK_MPI_VENC_StopRecvFrame(COMBO_JPEG_CHN);
    if (ret != RK_SUCCESS) {
        RK_LOGE("Failed to stop JPEG recv: %x", ret);
    }

    ret = RK_MPI_VENC_DestroyChn(COMBO_JPEG_CHN);
    if (ret != RK_SUCCESS) {
        RK_LOGE("Failed to destroy JPEG chn: %x, retrying once", ret);
        usleep(20000);
        ret = RK_MPI_VENC_DestroyChn(COMBO_JPEG_CHN);
    }
    if (ret != RK_SUCCESS) {
        // The channel still exists, so keep jpeg_initialized set: combo_jpeg_init()
        // must not create a second channel on top of the surviving one.
        RK_LOGE("Failed to destroy JPEG chn: %x, keeping channel state", ret);
        return ret;
    }

    jpeg_initialized = false;
    return ret;
}

static void *combo_jpeg_get_stream(void *arg)
{
    (void)arg;
    FILE *fp = NULL;
    VENC_STREAM_S stFrame;
    VENC_RECV_PIC_PARAM_S stRecvParam;
    int failCount = 0;

    // Report a stuck encoder instead of waiting blind, but keep waiting: returning
    // here would leave a dead thread behind and video_jpeg_get() would then skip
    // creating a new one (see jpeg_thread_join()).
    int waited_ms = 0;
    while (!venc_running) {
        if (waited_ms >= JPEG_VENC_WAIT_MAX_MS) {
            pthread_mutex_lock(&jpeg_mutex);
            bool pending = jpeg_mode == JPEG_SINGLE_SHOT && jpeg_trigger;
            jpeg_trigger = false;
            pthread_mutex_unlock(&jpeg_mutex);
            if (pending) {
                report_jpeg_failed("VENC not running");
            }
            waited_ms = 0;
        }
        usleep(10000);
        waited_ms += 10;
    }

    stFrame.pstPack = malloc(sizeof(VENC_PACK_S));
    if (!stFrame.pstPack) {
        RK_LOGE("malloc for stFrame.pstPack failed");
        return NULL;
    }

    while (venc_running) {
        pthread_mutex_lock(&jpeg_mutex);
        if (jpeg_mode == JPEG_SINGLE_SHOT && !jpeg_trigger) {
            pthread_mutex_unlock(&jpeg_mutex);
            usleep(10000);
            continue;
        }
        if (!jpeg_running) {
            pthread_mutex_unlock(&jpeg_mutex);
            usleep(10000);
            continue;
        }
        pthread_mutex_unlock(&jpeg_mutex);

        int s32Ret = RK_MPI_VENC_GetStream(COMBO_JPEG_CHN, &stFrame, 2000);
        if (s32Ret == RK_SUCCESS) {
            failCount = 0;
            void *pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);

            const char *path = "/userdata/picokvm/screenshot/kvm_screenshot.jpg";
            fp = fopen(path, "wb");
            if (fp) {
                fwrite(pData, 1, stFrame.pstPack->u32Len, fp);
                fflush(fp);
                fclose(fp);
                fp = NULL;
                RK_LOGD("JPEG snapshot saved to %s", path);
                // notify Go side that the file is ready
                report_jpeg_ready(path);
            } else {
                RK_LOGE("Failed to open %s: %s", path, strerror(errno));
                report_jpeg_failed("cannot write snapshot file");
            }

            RK_MPI_VENC_ReleaseStream(COMBO_JPEG_CHN, &stFrame);

            pthread_mutex_lock(&jpeg_mutex);
            if (jpeg_mode == JPEG_SINGLE_SHOT) {
                jpeg_trigger = false;
                RK_MPI_VENC_StopRecvFrame(COMBO_JPEG_CHN);
            } else {
                RK_MPI_VENC_StopRecvFrame(COMBO_JPEG_CHN);
                memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
                stRecvParam.s32RecvPicNum = 1;
                RK_MPI_VENC_StartRecvFrame(COMBO_JPEG_CHN, &stRecvParam);
            }
            pthread_mutex_unlock(&jpeg_mutex);
        } else {
            // Never retry in a tight loop: this device is single-core.
            if (failCount++ == 0) {
                RK_LOGE("RK_MPI_VENC_GetStream JPEG failed: %x", s32Ret);
            }
            if (failCount >= JPEG_GET_STREAM_MAX_FAILS) {
                // give up on the pending single-shot trigger instead of retrying forever
                pthread_mutex_lock(&jpeg_mutex);
                failCount = 0;
                bool dropped = false;
                if (jpeg_mode == JPEG_SINGLE_SHOT && jpeg_trigger) {
                    jpeg_trigger = false;
                    dropped = true;
                }
                pthread_mutex_unlock(&jpeg_mutex);
                if (dropped) {
                    // Somebody is waiting on a request that will never produce a file.
                    report_jpeg_failed("VENC_GetStream failed");
                }
                usleep(100000);
            } else {
                usleep(10000);
            }
        }
    }

    if (stFrame.pstPack) {
        free(stFrame.pstPack);
        stFrame.pstPack = NULL;
    }
    return NULL;
}

int video_jpeg_capture(JpegMode mode)
{
    if (!streaming_flag || !detected_signal) {
        RK_LOGE("Video streaming not active");
        return -1;
    }

    pthread_mutex_lock(&jpeg_mutex);
    if (jpeg_thread == NULL) {
        jpeg_thread = malloc(sizeof(pthread_t));
        if (pthread_create(jpeg_thread, NULL, combo_jpeg_get_stream, NULL) != 0) {
            RK_LOGE("Failed to create jpeg_thread");
            free(jpeg_thread);
            jpeg_thread = NULL;
            pthread_mutex_unlock(&jpeg_mutex);
            return -1;
        }
        RK_LOGD("JPEG thread created");
    }

    jpeg_mode = mode;
    jpeg_running = true;
    pthread_mutex_unlock(&jpeg_mutex);

    return 0;
}

int video_jpeg_get(void)
{
    if (!streaming_flag || !detected_signal) {
        RK_LOGE("Video streaming not active");
        return -1;
    }

    pthread_mutex_lock(&jpeg_mutex);
    if (jpeg_thread == NULL) {
        jpeg_thread = malloc(sizeof(pthread_t));
        if (pthread_create(jpeg_thread, NULL, combo_jpeg_get_stream, NULL) != 0) {
            RK_LOGE("Failed to create jpeg_thread");
            free(jpeg_thread);
            jpeg_thread = NULL;
            pthread_mutex_unlock(&jpeg_mutex);
            return -1;
        }
        RK_LOGD("JPEG thread created");
    }

    jpeg_trigger = true;
    jpeg_mode = JPEG_SINGLE_SHOT;
    jpeg_running = true;

    if (jpeg_initialized) {
        VENC_RECV_PIC_PARAM_S stRecvParam;
        memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
        stRecvParam.s32RecvPicNum = 1;
        RK_MPI_VENC_StartRecvFrame(COMBO_JPEG_CHN, &stRecvParam);
    }

    pthread_mutex_unlock(&jpeg_mutex);

    if (!jpeg_initialized) {
        // Nothing will feed the channel, so the trigger above can never complete.
        // Reported outside the lock: write_json() may block on the control socket.
        report_jpeg_failed("JPEG channel not initialized");
    }

    return 0;
}

void video_jpeg_stop(void)
{
    pthread_mutex_lock(&jpeg_mutex);
    jpeg_running = false;
    jpeg_trigger = false;
    pthread_mutex_unlock(&jpeg_mutex);

    jpeg_thread_join();
}

struct buffer
{
    struct v4l2_plane plane_buffer;
    MB_BLK mb_blk;
};

const int input_buffer_count = 3;

static int32_t buf_init()
{
    MB_POOL_CONFIG_S stMbPoolCfg;
    memset(&stMbPoolCfg, 0, sizeof(MB_POOL_CONFIG_S));
    stMbPoolCfg.u64MBSize = 1920 * 1080 * 3; // max resolution
    stMbPoolCfg.u32MBCnt = input_buffer_count;
    stMbPoolCfg.enAllocType = MB_ALLOC_TYPE_DMA;
    stMbPoolCfg.bPreAlloc = RK_TRUE;
    memPool = RK_MPI_MB_CreatePool(&stMbPoolCfg);
    if (memPool == MB_INVALID_POOLID)
    {
        return -1;
    }
    printf("Created memory pool\n");

    return RK_SUCCESS;
}

pthread_t *format_thread = NULL;
pthread_t *preview_thread = NULL;
int preview_client_fd = -1;
pthread_mutex_t preview_mutex = PTHREAD_MUTEX_INITIALIZER;

int video_client_fd = 0;

int connect_video_client(const char *path)
{
    int client_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (client_fd == -1)
    {
        perror("can not create socket");
        return -1;
    }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(struct sockaddr_un));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path, sizeof(addr.sun_path) - 1);

    if (connect(client_fd, (struct sockaddr *)&addr, sizeof(addr)) == -1)
    {
        perror("can not connect to video socket");
        close(client_fd);
        return -1;
    }
    video_client_fd = client_fd;
    return 0;
}

int socket_send_frame(const uint8_t *frame, ssize_t len)
{
    if (video_client_fd <= 0)
    {
        return -1;
    }
    return send(video_client_fd, frame, len, 0);
}

int video_init()
{
    if (connect_video_client("/var/run/kvm_video.sock") != 0)
    {
        printf("can not connect to video socket\n");
        return -1;
    }
    printf("Connected to video socket\n");

    if (sub_dev_fd < 0)
    {
        sub_dev_fd = open(SUB_DEV, O_RDWR);
        if (sub_dev_fd < 0)
        {
            printf("failed to open control sub device %s: %s\n", SUB_DEV, strerror(errno));
            return errno;
        }
        printf("Opened control sub device %s\n", SUB_DEV);
    }

    int32_t ret = buf_init();
    if (ret != RK_SUCCESS)
    {
        RK_LOGE("buf_init failed with error: %d", ret);
        return ret;
    }
    printf("buf_init completed successfully\n");

    // YOLOv5 model will be loaded on-demand when video_set_yolo_enable(true) is called

    format_thread = malloc(sizeof(pthread_t));
    pthread_create(format_thread, NULL, run_detect_format, NULL);

    preview_thread = malloc(sizeof(pthread_t));
    pthread_create(preview_thread, NULL, run_preview_socket_thread, NULL);

    return RK_SUCCESS;
}

// static int32_t venc_set_param(int32_t bitrate, int32_t max_bitrate, int32_t width, int32_t height)
// {

//     VENC_CHN_ATTR_S stAttr;
//     populate_venc_attr(&stAttr, bitrate, max_bitrate, width, height);
//     VENC_CHN_PARAM_S stParam;
//     memset(&stParam, 0, sizeof(VENC_CHN_PARAM_S));

//     RK_MPI_VENC_StopRecvFrame(VENC_CHANNEL);

//     int32_t ret = RK_MPI_VENC_SetChnParam(VENC_CHANNEL, &stAttr);
//     if (ret < 0)
//     {
//         RK_LOGE("error RK_MPI_VENC_SetChnParam, %d", ret);
//         return ret;
//     }
//     VENC_RECV_PIC_PARAM_S stRecvParam;
//     memset(&stRecvParam, 0, sizeof(VENC_RECV_PIC_PARAM_S));
//     stRecvParam.s32RecvPicNum = -1;
//     ret = RK_MPI_VENC_StartRecvFrame(VENC_CHANNEL, &stRecvParam);
//     if (ret < 0)
//     {
//         RK_LOGE("error RK_MPI_VENC_StartRecvFrame, %d", ret);
//         return ret;
//     }

//     return RK_SUCCESS;
// }

/**
 * @brief Continuously reads encoded video streams and sends them over unix socket.
 *
 * @param arg Unused parameter (void pointer for thread compatibility)
 * @return NULL Always returns NULL
 */
static void *venc_read_stream(void *arg)
{
    (void)arg;
    void *pData = RK_NULL;
    int loopCount = 0;
    int s32Ret;

    VENC_STREAM_S stFrame;
    stFrame.pstPack = malloc(sizeof(VENC_PACK_S));
    while (venc_running)
    {
        // printf("RK_MPI_VENC_GetStream\n");
        s32Ret = RK_MPI_VENC_GetStream(VENC_CHANNEL, &stFrame, 200); // blocks max 200ms
        if (s32Ret == RK_SUCCESS)
        {
            RK_U64 nowUs = get_us();
            // printf("chn:0, loopCount:%d enc->seq:%d wd:%d pts=%llu delay=%lldus\n",
            //        loopCount, stFrame.u32Seq, stFrame.pstPack->u32Len,
            //        stFrame.pstPack->u64PTS, nowUs - stFrame.pstPack->u64PTS);
            pData = RK_MPI_MB_Handle2VirAddr(stFrame.pstPack->pMbBlk);
            socket_send_frame(pData, (ssize_t)stFrame.pstPack->u32Len);
            s32Ret = RK_MPI_VENC_ReleaseStream(VENC_CHANNEL, &stFrame);
            if (s32Ret != RK_SUCCESS)
            {
                RK_LOGE("RK_MPI_VENC_ReleaseStream fail %x", s32Ret);
            }
            loopCount++;
        }
        else
        {
            if (s32Ret == RK_ERR_VENC_BUF_EMPTY)
            {
                continue;
            }
            RK_LOGE("RK_MPI_VENC_GetStream fail %x", s32Ret);
            break;
        }
    }
    printf("exiting venc_read_stream\n");
    free(stFrame.pstPack);
    return NULL;
}

pthread_t *streaming_thread = NULL;

static void *run_preview_socket_thread(void *arg) {
    int server_fd, client_fd;
    struct sockaddr_un address;
    const char *socket_path = "/var/run/kvm_preview.sock";

    server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (server_fd == -1) {
        perror("preview socket");
        return NULL;
    }

    memset(&address, 0, sizeof(struct sockaddr_un));
    address.sun_family = AF_UNIX;
    strncpy(address.sun_path, socket_path, sizeof(address.sun_path) - 1);
    unlink(socket_path);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) == -1) {
        perror("preview bind");
        return NULL;
    }

    if (listen(server_fd, 5) == -1) {
        perror("preview listen");
        return NULL;
    }

    printf("Preview socket listening on %s\n", socket_path);

    while (!should_exit) {
        client_fd = accept(server_fd, NULL, NULL);
        if (client_fd == -1) {
            if (should_exit) break;
            perror("preview accept");
            continue;
        }

        printf("Preview client connected\n");
        pthread_mutex_lock(&preview_mutex);
        if (preview_client_fd != -1) {
            close(preview_client_fd);
        }
        preview_client_fd = client_fd;
        pthread_mutex_unlock(&preview_mutex);
    }
    close(server_fd);
    unlink(socket_path);
    return NULL;
}

void write_buffer_to_file(const uint8_t *buffer, size_t length, const char *filename)
{
    FILE *file = fopen(filename, "wb");
    fwrite(buffer, 1, length, file);
    fclose(file);
}

void *run_video_stream(void *arg)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    while (streaming_flag)
    {
        if (detected_signal == false)
        {
            usleep(100000);
            continue;
        }

        int video_dev_fd = open(VIDEO_DEV, O_RDWR);
        if (video_dev_fd < 0)
        {
            printf("failed to open video capture device %s: %s\n", VIDEO_DEV, strerror(errno));
            usleep(1000000);
            continue;
        }
        printf("Opened video capture device %s\n", VIDEO_DEV);

        uint32_t width = detected_width;
        uint32_t height = detected_height;
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(struct v4l2_format));
        fmt.type = type;
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;

        if (ioctl(video_dev_fd, VIDIOC_S_FMT, &fmt) < 0)
        {
            perror("Set format fail");
            usleep(100000); // Sleep for 100 milliseconds
            close(video_dev_fd);
            continue;
        }

        struct v4l2_buffer buf;

        struct v4l2_requestbuffers req;
        req.count = input_buffer_count;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_DMABUF;

        if (ioctl(video_dev_fd, VIDIOC_REQBUFS, &req) < 0)
        {
            perror("VIDIOC_REQBUFS failed");
            return errno;
        }
        printf("VIDIOC_REQBUFS successful\n");

        struct buffer buffers[3] = {};
        printf("Allocated buffers\n");

        for (int i = 0; i < input_buffer_count; i++)
        {
            struct v4l2_plane *planes_buffer = &buffers[i].plane_buffer;
            memset(planes_buffer, 0, sizeof(struct v4l2_plane));

            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_DMABUF;
            buf.m.planes = planes_buffer;
            buf.length = 1;
            buf.index = i;

            if (-1 == ioctl(video_dev_fd, VIDIOC_QUERYBUF, &buf))
            {
                perror("VIDIOC_QUERYBUF failed");
                req.count = i;
                return errno;
            }
            printf("VIDIOC_QUERYBUF successful for buffer %d\n", i);

            printf("plane: length = %d\n", planes_buffer->length);
            printf("plane: offset = %d\n", planes_buffer->m.mem_offset);

            MB_BLK blk = RK_MPI_MB_GetMB(memPool, (planes_buffer)->length, RK_TRUE);
            if (blk == NULL)
            {
                RK_LOGE("get mb blk failed!");
                return -1;
            }
            printf("Got memory block for buffer %d\n", i);

            buffers[i].mb_blk = blk;

            RK_S32 buf_fd = (RK_MPI_MB_Handle2Fd(blk));
            if (buf_fd < 0)
            {
                RK_LOGE("RK_MPI_MB_Handle2Fd failed!");
                return -1;
            }
            printf("Converted memory block to file descriptor for buffer %d\n", i);
            planes_buffer->m.fd = buf_fd;
        }

        for (int i = 0; i < input_buffer_count; ++i)
        {
            struct v4l2_buffer buf;
            memset(&buf, 0, sizeof(buf));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_DMABUF;
            buf.length = 1;
            buf.index = i;
            buf.m.planes = &buffers[i].plane_buffer;
            if (ioctl(video_dev_fd, VIDIOC_QBUF, &buf) < 0)
            {
                perror("VIDIOC_QBUF failed");
                return errno;
            }
            printf("VIDIOC_QBUF successful for buffer %d\n", i);
        }

        if (ioctl(video_dev_fd, VIDIOC_STREAMON, &type) < 0)
        {
            perror("VIDIOC_STREAMON failed");
            goto cleanup;
        }

        struct v4l2_plane tmp_plane;

        // Set VENC parameters
        int32_t bitrate = calculate_bitrate(quality_factor, width, height);
        RK_CODEC_ID_E enType = encodec_type;
        RK_S32 ret = venc_start(enType, bitrate, bitrate * 2, width, height, &video_rc_config);
        if (ret != RK_SUCCESS)
        {
        RK_LOGE("Set VENC parameters failed with %#x", ret);
        goto cleanup;
    }

        overlay_init(width, height, VENC_CHANNEL);

        fd_set fds;
        struct timeval tv;
        int r;
        uint32_t num = 0;
        int select_timeout_count = 0;
        VIDEO_FRAME_INFO_S stFrame;

        while (streaming_flag)
        {
            FD_ZERO(&fds);
            FD_SET(video_dev_fd, &fds);
            tv.tv_sec = 0;
            tv.tv_usec = 500000; // 500ms timeout

            r = select(video_dev_fd + 1, &fds, NULL, NULL, &tv);
            if (r == 0)
            {
                select_timeout_count++;
                printf("select timeout (count: %d)\n", select_timeout_count);

                if (access(HDMI_STATUS_FILE, F_OK) == 0)
                {
                  bool hdmi_connected = check_hdmi_connection_from_debugfs();
                  if (!hdmi_connected)
                  {
                    detected_signal = false;
                    report_video_format(false, "no_signal", 0, 0, 0);
                    break;
                  }
                }

                if (select_timeout_count >= 3)
                {
                    printf("No data received for 1.5 seconds, signal lost\n");
                    detected_signal = false;
                    report_video_format(false, "no_signal", 0, 0, 0);
                    break;
                }
                continue;
            }
            else
            {
                select_timeout_count = 0;
            }
            if (r == -1)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                perror("select in video streaming");
                break;
            }
            memset(&buf, 0, sizeof(buf));
            memset(&tmp_plane, 0, sizeof(tmp_plane));
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_DMABUF;
            buf.m.planes = &tmp_plane;
            buf.length = 1;
            if (ioctl(video_dev_fd, VIDIOC_DQBUF, &buf) < 0)
            {
                perror("VIDIOC_DQBUF failed");
                break;
            }

            pthread_mutex_lock(&preview_mutex);
            if (preview_client_fd != -1) {
                // Calculate proportional height to fit width 240
                int preview_w = 240;
                int preview_h = 240;
                if (width > 0) {
                    preview_h = (int)((float)height * preview_w / width);
                    // Ensure height is even for YUV requirements
                    preview_h = (preview_h + 1) & ~1;
                    if (preview_h > 240) preview_h = 240; // Should not happen for 16:9
                }

                static uint8_t rgb_preview_buf[240 * 240 * 2];

                uint16_t *ptr = (uint16_t *)rgb_preview_buf;
                for (int i = 0; i < 240 * 240; i++) {
                    ptr[i] = 0x0000;
                }

                int y_offset = (240 - preview_h) / 2;
                uint8_t *dst_ptr = rgb_preview_buf + (y_offset * 240 * 2);

                if (yuyvfd_to_rgb565_resized(tmp_plane.m.fd, width, height, dst_ptr, preview_w, preview_h) == 0) {
                     send(preview_client_fd, rgb_preview_buf, sizeof(rgb_preview_buf), MSG_DONTWAIT);
                }
            }
            pthread_mutex_unlock(&preview_mutex);

            // printf("got frame, bytesused = %d\n", tmp_plane.bytesused);
            memset(&stFrame, 0, sizeof(VIDEO_FRAME_INFO_S));
            MB_BLK blk = RK_NULL;
            blk = RK_MPI_MMZ_Fd2Handle(tmp_plane.m.fd);
            assert(blk != RK_NULL);
            stFrame.stVFrame.pMbBlk = blk;
            stFrame.stVFrame.u32Width = width;
            stFrame.stVFrame.u32Height = height;
            // stFrame.stVFrame.u32VirWidth = (width + 15) & (~15);
            // stFrame.stVFrame.u32VirHeight = (height + 15) & (~15);
            stFrame.stVFrame.u32VirWidth = RK_ALIGN_2(width);
            stFrame.stVFrame.u32VirHeight = RK_ALIGN_2(height);
            stFrame.stVFrame.u32TimeRef = num; // frame number
            stFrame.stVFrame.u64PTS = get_us();
            stFrame.stVFrame.enPixelFormat = RK_FMT_YUV422_YUYV;
            stFrame.stVFrame.u32FrameFlag |= 0;
            stFrame.stVFrame.enCompressMode = COMPRESS_MODE_NONE;
            bool retried = false;
        // if (num == 100) {
        //     RK_VOID *pData = RK_MPI_MB_Handle2VirAddr(stFrame.stVFrame.pMbBlk);
        //     if (pData) {
        //         size_t frameSize = tmp_plane.bytesused; // Use the actual size reported by the driver
        //         write_buffer_to_file(pData, frameSize, "/userdata/banana.raw");
        //         printf("Frame 100 written to /userdata/banana.raw\n");
        //     } else {
        //         printf("Failed to get virtual address for frame 100\n");
        //     }
        // }
        retry_send_frame:
            if (RK_MPI_VENC_SendFrame(VENC_CHANNEL, &stFrame, 2000) != RK_SUCCESS)
            {
                if (retried == true)
                {
                    RK_LOGE("RK_MPI_VENC_SendFrame retry failed");
                }
                else
                {
                    RK_LOGE("RK_MPI_VENC_SendFrame failed,retrying");
                    retried = true;
                    usleep(1000llu);
                    goto retry_send_frame;
                }
            }

            num++;

            if (ioctl(video_dev_fd, VIDIOC_QBUF, &buf) < 0)
                printf("failture VIDIOC_QBUF\n");

            if (yolo_running) {
                pthread_mutex_lock(&yolo_mutex);
                if (!yolo_has_frame) {
                    if (yolo_model_w > 0 && yolo_model_h > 0) {
                        int out_fd = -1; void* out_va = NULL;
                        int r = yuyvfd_to_rgb_resized_fd(tmp_plane.m.fd, (int)width, (int)height, &out_fd, &out_va, yolo_model_w, yolo_model_h);
                        if (r == 0 && out_fd >= 0 && out_va) {
                            yolo_fd = out_fd;
                            yolo_va = out_va;
                            yolo_model_buf_size = (size_t)yolo_model_w * (size_t)yolo_model_h * (size_t)yolo_model_c;
                            yolo_has_frame = true;
                            pthread_cond_signal(&yolo_cond);
                        }
                    }
                }
                pthread_mutex_unlock(&yolo_mutex);
            }
        }
    cleanup:
        if (ioctl(video_dev_fd, VIDIOC_STREAMOFF, &type) < 0)
        {
            perror("VIDIOC_STREAMOFF failed");
        }

        venc_stop();
        overlay_deinit(VENC_CHANNEL);

        for (int i = 0; i < input_buffer_count; i++)
        {
            if (buffers[i].mb_blk != NULL)
            {
                RK_MPI_MB_ReleaseMB((buffers + i)->mb_blk);
            }
        }

        close(video_dev_fd);
    }

    return NULL;
}

void video_shutdown()
{
    if (should_exit == true)
    {
        printf("shutting down in progress already\n");
        return;
    }
    video_stop_streaming();
    // if (buffers != NULL) {
    //     for (int i = 0; i < input_buffer_count; i++) {
    //         if ((buffers + i)->mb_blk != NULL) {
    //             RK_MPI_MB_ReleaseMB((buffers + i)->mb_blk);
    //         }
    //         free((buffers + i)->planes_buffer);
    //     }
    //     free(buffers);
    // }
    should_exit = true;
    if (sub_dev_fd > 0)
    {
        shutdown(sub_dev_fd, SHUT_RDWR);
        // close(sub_dev_fd);
        printf("Closed sub_dev_fd\n");
    }

    if (memPool != MB_INVALID_POOLID)
    {
        RK_MPI_MB_DestroyPool(memPool);
    }
    printf("Destroyed memory pool\n");
    if (yolo_yuyv) { free(yolo_yuyv); yolo_yuyv = NULL; yolo_yuyv_size = 0; }
    if (yolo_rgb) { free(yolo_rgb); yolo_rgb = NULL; yolo_rgb_size = 0; }
    if (yolo_model_buf) { free(yolo_model_buf); yolo_model_buf = NULL; yolo_model_buf_size = 0; }
    yolo_deinit();
    // if (format_thread != NULL) {
    //     pthread_join(*format_thread, NULL);
    //     free(format_thread);
    //     format_thread = NULL;
    // }
    // printf("Joined format detection thread\n");
}

// TODO: mutex?

void video_start_streaming()
{
    pthread_mutex_lock(&streaming_mutex);
    if (streaming_thread != NULL)
    {
        printf("video streaming already started\n");
        pthread_mutex_unlock(&streaming_mutex);
        return;
    }
    streaming_thread = malloc(sizeof(pthread_t));
    assert(streaming_thread != NULL);
    streaming_flag = true;
    pthread_create(streaming_thread, NULL, run_video_stream, NULL);
    pthread_mutex_unlock(&streaming_mutex);
}

void video_stop_streaming()
{
    pthread_mutex_lock(&streaming_mutex);
    if (streaming_thread != NULL)
    {
        streaming_flag = false;
        pthread_mutex_unlock(&streaming_mutex);
        pthread_join(*streaming_thread, NULL);
        free(streaming_thread);
        streaming_thread = NULL;
        printf("video streaming stopped\n");
    }
    else
    {
        pthread_mutex_unlock(&streaming_mutex);
    }
    if (yolo_thread != NULL) {
        yolo_running = false;
        pthread_mutex_lock(&yolo_mutex);
        pthread_cond_signal(&yolo_cond);
        pthread_mutex_unlock(&yolo_mutex);
        pthread_join(*yolo_thread, NULL);
        free(yolo_thread);
        yolo_thread = NULL;
    }
}

void *run_detect_format(void *arg)
{
    struct v4l2_event_subscription sub;
    struct v4l2_event ev;
    struct v4l2_dv_timings dv_timings;
    bool hdmi_connected = false;

    memset(&sub, 0, sizeof(sub));
    sub.type = V4L2_EVENT_SOURCE_CHANGE;
    if (ioctl(sub_dev_fd, VIDIOC_SUBSCRIBE_EVENT, &sub) == -1)
    {
        perror("Cannot subscribe to event");
        goto exit;
    }

    while (!should_exit)
    {
        bool signal_detected = false;

        if (access(HDMI_STATUS_FILE, F_OK) == 0)
        {
            hdmi_connected = check_hdmi_connection_from_debugfs();
            if (!hdmi_connected)
            {
                detected_signal = false;
                printf("HDMI status: disconnected (from debugfs)\n");
                report_video_format(false, "no_signal", 0, 0, 0);
                goto wait_event;
            }

            memset(&dv_timings, 0, sizeof(dv_timings));
            if (ioctl(sub_dev_fd, VIDIOC_QUERY_DV_TIMINGS, &dv_timings) == 0)
            {
                if (dv_timings.bt.width > 0 && dv_timings.bt.height > 0)
                {
                    bool timing_valid = true;
                    if (dv_timings.bt.hsync == 0 || dv_timings.bt.vsync == 0)
                    {
                        printf("WARNING: Invalid timing - hs:%d, vs:%d\n",
                               dv_timings.bt.hsync, dv_timings.bt.vsync);
                        timing_valid = false;
                    }

                    if (dv_timings.bt.pixelclock == 0 || dv_timings.bt.pixelclock > 600000000)
                    {
                        printf("WARNING: Invalid pixelclock: %llu\n", dv_timings.bt.pixelclock);
                        timing_valid = false;
                    }

                    if (timing_valid)
                    {
                        signal_detected = true;
                    }
                    else
                    {
                        printf("HDMI connected but timing invalid\n");
                        detected_signal = false;
                        report_video_format(false, "no_signal", 0, 0, 0);
                    }
                }
                else
                {
                    printf("HDMI connected but invalid resolution: %dx%d\n", dv_timings.bt.width, dv_timings.bt.height);
                    detected_signal = false;
                    report_video_format(false, "no_signal", 0, 0, 0);
                }
            }
            else
            {
                printf("HDMI connected but ioctl failed: %s\n", strerror(errno));
                detected_signal = false;
                report_video_format(false, "no_signal", 0, 0, 0);
            }
        }
        else
        {
            memset(&dv_timings, 0, sizeof(dv_timings));
            if (ioctl(sub_dev_fd, VIDIOC_QUERY_DV_TIMINGS, &dv_timings) != 0)
            {
                detected_signal = false;
                if (errno == ENOLINK)
                {
                    printf("HDMI status: no signal\n");
                    report_video_format(false, "no_signal", 0, 0, 0);
                }
                else if (errno == ENOLCK)
                {
                    printf("HDMI status: no lock\n");
                    report_video_format(false, "no_lock", 0, 0, 0);
                }
                else if (errno == ERANGE)
                {
                    printf("HDMI status: out of range\n");
                    report_video_format(false, "out_of_range", 0, 0, 0);
                }
                else
                {
                    perror("error VIDIOC_QUERY_DV_TIMINGS");
                    sleep(1);
                    continue;
                }
            }
            else
            {
                if (dv_timings.bt.width > 0 && dv_timings.bt.height > 0)
                {
                    signal_detected = true;
                }
            }
        }

        if (signal_detected)
        {
            printf("Active width: %d\n", dv_timings.bt.width);
            printf("Active height: %d\n", dv_timings.bt.height);
            double frames_per_second = (double)dv_timings.bt.pixelclock /
                                      ((dv_timings.bt.height + dv_timings.bt.vfrontporch + dv_timings.bt.vsync +
                                        dv_timings.bt.vbackporch) *
                                        (dv_timings.bt.width + dv_timings.bt.hfrontporch + dv_timings.bt.hsync +
                                        dv_timings.bt.hbackporch));
            printf("Frames per second: %.2f fps\n", frames_per_second);

            bool resolution_changed = (detected_width != (uint32_t)dv_timings.bt.width ||
                                       detected_height != (uint32_t)dv_timings.bt.height);
            bool was_no_signal = !detected_signal;
            detected_width = dv_timings.bt.width;
            detected_height = dv_timings.bt.height;
            detected_signal = true;
            report_video_format(true, NULL, detected_width, detected_height, frames_per_second);

            if (resolution_changed || was_no_signal)
            {
                if (streaming_flag == true && !was_no_signal)
                {
                    printf("resolution changed, restarting video streaming\n");
                    video_stop_streaming();
                    usleep(200000);
                }
                video_start_streaming();
            }
            else
            {
                printf("signal unchanged at %dx%d, no restart needed\n", detected_width, detected_height);
            }
        }
        else if (hdmi_connected)
        {
            detected_signal = false;
            printf("HDMI connected but no valid signal\n");
            report_video_format(false, "no_signal", 0, 0, 0);
        }

wait_event:


        memset(&ev, 0, sizeof(ev));
        if (ioctl(sub_dev_fd, VIDIOC_DQEVENT, &ev) != 0)
        {
            perror("failed to VIDIOC_DQEVENT");
            break;
        }
        printf("New event of type %u\n", ev.type);
        if (ev.type != V4L2_EVENT_SOURCE_CHANGE)
        {
            continue;
        }
        printf("source change detected!\n");
        usleep(500000); // 500ms
    }
exit:
    close(sub_dev_fd);
    return NULL;
}


void video_set_quality_factor(float factor)
{
    quality_factor = factor;

    // TODO: update venc bitrate without stopping streaming

    if (streaming_flag == true)
    {
        printf("restarting on going video streaming due to quality factor change\n");
        video_stop_streaming();
        video_start_streaming();
    }
}

void video_set_encodec_type(RK_CODEC_ID_E type)
{
    encodec_type = type;

    if (streaming_flag == true)
    {
        printf("restarting on going video streaming due to encodec type change\n");
        video_stop_streaming();
        video_start_streaming();
    }
}

void video_set_yolo_enable(int enable)
{
    if (enable)
    {
        if (!yolo_running)
        {
            // Load model on-demand if not already loaded
            if (yolo_model_w == 0 || yolo_model_h == 0) {
                if (yolo_init("/userdata/picokvm/model/yolov5.rknn") != 0) {
                    printf("yolov5 init failed\n");
                    return;
                }
                yolo_input_shape(&yolo_model_h, &yolo_model_w, &yolo_model_c);
            }
            yolo_running = true;
            if (yolo_thread == NULL)
            {
                yolo_thread = malloc(sizeof(pthread_t));
                pthread_create(yolo_thread, NULL, yolo_infer_thread, NULL);
            }
        }
    }
    else
    {
        if (yolo_running)
        {
            yolo_running = false;
            pthread_mutex_lock(&yolo_mutex);
            pthread_cond_signal(&yolo_cond);
            pthread_mutex_unlock(&yolo_mutex);
            if (yolo_thread)
            {
                pthread_join(*yolo_thread, NULL);
                free(yolo_thread);
                yolo_thread = NULL;
            }
            overlay_draw_detections(yolo_model_w, yolo_model_h, NULL, 0, 0x00FF00FF);
            // Unload model to free NPU resources
            yolo_deinit();
            yolo_model_w = 0;
            yolo_model_h = 0;
            yolo_model_c = 0;
        }
    }
}

void video_set_rc(const VideoRcConfig *rc)
{
    video_rc_config = *rc;

    if (streaming_flag == true)
    {
        printf("restarting video streaming due to RC config change\n");
        video_stop_streaming();
        video_start_streaming();
    }
}

void video_get_rc(VideoRcConfig *rc)
{
    *rc = video_rc_config;
}

static void* yolo_infer_thread(void* arg) {
    yolo_input_shape(&yolo_model_h, &yolo_model_w, &yolo_model_c);
    size_t expect = (size_t)yolo_model_h * (size_t)yolo_model_w * (size_t)yolo_model_c;
    yolo_model_buf = (uint8_t*)malloc(expect);
    yolo_model_buf_size = expect;
    while (yolo_running) {
        pthread_mutex_lock(&yolo_mutex);
        while (!yolo_has_frame && yolo_running) {
            pthread_cond_wait(&yolo_cond, &yolo_mutex);
        }
        if (!yolo_running) {
            pthread_mutex_unlock(&yolo_mutex);
            break;
        }
        pthread_mutex_unlock(&yolo_mutex);
        if (yolo_fd >= 0 && yolo_va) {
            if (yolo_bound_fd != yolo_fd) {
                if (yolo_bind_input_fd(yolo_fd, yolo_va, yolo_model_buf_size) == 0) {
                    yolo_bound_fd = yolo_fd;
                }
            }
            yolo_det_t dets[YOLO_MAX_DETS];
            int count = yolo_run(dets, YOLO_MAX_DETS);
            //printf("yolov5 detections: %d\n", count);
            int print_n = count < YOLO_MAX_DETS ? count : YOLO_MAX_DETS;
            for (int i = 0; i < print_n; ++i) {
                const char* name = yolo_cls_name(dets[i].cls_id);
                //printf("det %d: %s %.2f [%d,%d,%d,%d]\n", i, name, dets[i].conf, dets[i].left, dets[i].top, dets[i].right, dets[i].bottom);
            }
            overlay_draw_detections(yolo_model_w, yolo_model_h, dets, print_n, 0x00FF00FF);
        }
        pthread_mutex_lock(&yolo_mutex);
        yolo_has_frame = false;
        pthread_mutex_unlock(&yolo_mutex);
    }
    return NULL;
}
