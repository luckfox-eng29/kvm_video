#include <stdio.h>
#include <string.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include <stdint.h>
#include <stdarg.h>
#include <fcntl.h>
#include "frozen.h"
#include "video.h"
#include "edid.h"

typedef struct
{
    bool ready;
    const char *error;
    u_int16_t width;
    u_int16_t height;
    double frame_per_second;
} state_t;

state_t state;

struct sockaddr_un addr;

int ctrl_client_fd = 0;

int write_json(int fd, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    char *json_str = json_vasprintf(fmt, ap);
    va_end(ap);
    if (json_str == NULL)
    {
        return -1;
    }
    int written = write(fd, json_str, strlen(json_str));
    free(json_str);
    return written;
}

int write_json_error(int fd, int seq, const char *error)
{
    return write_json(fd, "{seq: %d, error: %Q, errno: %d}", seq, error, errno);
}

static int write_input_video_state(int fd)
{
    return write_json(fd, "{event: %Q, data: {ready: %B, error: %Q, width: %u, height:%u, frame_per_second: %f}}",
                      "video_input_state", state.ready, state.error, state.width, state.height, state.frame_per_second);
}

void report_jpeg_ready(const char *path)
{
    if (ctrl_client_fd > 0)
    {
        write_json(ctrl_client_fd, "{event: %Q, data: {path: %Q}}", "jpeg_ready", path);
    }
}

void report_jpeg_failed(const char *reason)
{
    if (ctrl_client_fd > 0)
    {
        write_json(ctrl_client_fd, "{event: %Q, data: {error: %Q}}", "jpeg_failed", reason);
    }
}

void report_video_format(bool ready, const char *error, u_int16_t width, u_int16_t height, double frame_per_second)
{
    state.ready = ready;
    state.error = error;
    state.width = width;
    state.height = height;
    state.frame_per_second = frame_per_second;
    if (ctrl_client_fd > 0)
    {
        if (write_input_video_state(ctrl_client_fd) < 0)
        {
            perror("error writing video state");
        }
    }
}

/**
 * @brief Convert a hexadecimal string to an array of uint8_t bytes
 *
 * @param hex_str The input hexadecimal string
 * @param bytes The output byte array (must be pre-allocated)
 * @param max_len The maximum number of bytes that can be stored in the output array
 * @return int The number of bytes converted, or -1 on error
 */
int hex_to_bytes(const char *hex_str, uint8_t *bytes, size_t max_len)
{
    size_t hex_len = strnlen(hex_str, 4096);
    if (hex_len % 2 != 0 || hex_len / 2 > max_len)
    {
        return -1; // Invalid input length or insufficient output buffer
    }

    for (size_t i = 0; i < hex_len; i += 2)
    {
        char byte_str[3] = {hex_str[i], hex_str[i + 1], '\0'};
        char *end_ptr;
        long value = strtol(byte_str, &end_ptr, 16);

        if (*end_ptr != '\0' || value < 0 || value > 255)
        {
            return -1; // Invalid hexadecimal value
        }

        bytes[i / 2] = (uint8_t)value;
    }

    return hex_len / 2;
}

/**
 * @brief Convert an array of uint8_t bytes to a hexadecimal string, user must free the returned string
 *
 * @param bytes The input byte array
 * @param len The number of bytes in the input array
 * @return char* The output hexadecimal string (dynamically allocated, must be freed by the caller), or NULL on error
 */
const char *bytes_to_hex(const uint8_t *bytes, size_t len)
{
    if (bytes == NULL || len == 0)
    {
        return NULL;
    }

    char *hex_str = malloc(2 * len + 1); // Each byte becomes 2 hex chars, plus null terminator
    if (hex_str == NULL)
    {
        return NULL; // Memory allocation failed
    }

    for (size_t i = 0; i < len; i++)
    {
        snprintf(hex_str + (2 * i), 3, "%02x", bytes[i]);
    }

    hex_str[2 * len] = '\0'; // Ensure null termination
    return hex_str;
}

void *handle_client(void *arg)
{
    const int BUF_SIZE = 1024;
    char buf_in[BUF_SIZE];

    // TODO: fix memory leak caused by continue
    while (true)
    {
        ssize_t n = read(ctrl_client_fd, buf_in, BUF_SIZE);
        if (n > 0)
        {
            buf_in[n] = '\0';
            char *method;
            int32_t seq = -1;
            if (json_scanf(buf_in, n, "{action: %Q, seq: %d}", &method, &seq) > 0)
            {
                if (strcmp("start_video", method) == 0)
                {
                    video_start_streaming();
                    write_json(ctrl_client_fd, "{seq: %d}", seq);
                }
                else if (strcmp("stop_video", method) == 0)
                {
                    video_stop_streaming();
                    write_json(ctrl_client_fd, "{seq: %d}", seq);
                }
                else if (strcmp("set_video_quality_factor", method) == 0)
                {
                    float quality_factor;
                    if (json_scanf(buf_in, n, "{params: {quality_factor: %f}}", &quality_factor) > 0)
                    {
                        if (quality_factor < 0 || quality_factor > 1)
                        {
                            write_json_error(ctrl_client_fd, seq, "invalid quality factor");
                        }
                        else
                        {
                            video_set_quality_factor(quality_factor);
                            write_json(ctrl_client_fd, "{seq: %d}", seq);
                        }
                    }
                    else
                    {
                        write_json_error(ctrl_client_fd, seq, "invalid quality factor");
                    }
                }
                else if (strcmp("set_video_encodec_type", method) == 0)
                {
                    RK_CODEC_ID_E encodec_type;
                    char* encodec_type_str;
                    if (json_scanf(buf_in, n, "{params: {encodec_type: %Q}}", &encodec_type_str) > 0)
                    {
                        if (strcmp(encodec_type_str, "avc") == 0)
                        {
                            encodec_type = RK_VIDEO_ID_AVC;
                        }
                        else if (strcmp(encodec_type_str, "hevc") == 0)
                        {
                            encodec_type = RK_VIDEO_ID_HEVC;
                        }
                        else
                        {
                            write_json_error(ctrl_client_fd, seq, "invalid encodec type");
                            continue;
                        }
                        video_set_encodec_type(encodec_type);
                        write_json(ctrl_client_fd, "{seq: %d}", seq);
                    }
                    else
                    {
                        write_json_error(ctrl_client_fd, seq, "invalid encodec type");
                    }
                }
                else if (strcmp("set_yolo_enable", method) == 0)
                {
                    char enable_str[16];
                    if (json_scanf(buf_in, n, "{params: {enable: %s}}", enable_str) > 0)
                    {
                        video_set_yolo_enable(strcmp(enable_str, "true") == 0);
                        write_json(ctrl_client_fd, "{seq: %d}", seq);
                    }
                    else
                    {
                        write_json_error(ctrl_client_fd, seq, "invalid enable");
                    }
                }
                else if (strcmp("set_video_rc", method) == 0)
                {
                    VideoRcConfig rc;
                    memset(&rc, 0, sizeof(rc));

                    int parsed = json_scanf(buf_in, n,
                        "{params: {"
                        "h264: {"
                        "s32FirstFrameStartQp: %d,"
                        "u32StepQp: %u,"
                        "u32MinQp: %u,"
                        "u32MaxQp: %u,"
                        "u32MinIQp: %u,"
                        "u32MaxIQp: %u,"
                        "s32DeltIpQp: %d,"
                        "s32MaxReEncodeTimes: %d,"
                        "u32FrmMaxQp: %u,"
                        "u32FrmMinQp: %u,"
                        "u32FrmMinIQp: %u,"
                        "u32FrmMaxIQp: %u,"
                        "u32MotionStaticSwitchFrmQp: %u"
                        "},"
                        "h265: {"
                        "s32FirstFrameStartQp: %d,"
                        "u32StepQp: %u,"
                        "u32MinQp: %u,"
                        "u32MaxQp: %u,"
                        "u32MinIQp: %u,"
                        "u32MaxIQp: %u,"
                        "s32DeltIpQp: %d,"
                        "s32MaxReEncodeTimes: %d,"
                        "u32FrmMaxQp: %u,"
                        "u32FrmMinQp: %u,"
                        "u32FrmMinIQp: %u,"
                        "u32FrmMaxIQp: %u,"
                        "u32MotionStaticSwitchFrmQp: %u"
                        "}"
                        "}}",
                        &rc.h264.s32FirstFrameStartQp,
                        &rc.h264.u32StepQp,
                        &rc.h264.u32MinQp,
                        &rc.h264.u32MaxQp,
                        &rc.h264.u32MinIQp,
                        &rc.h264.u32MaxIQp,
                        &rc.h264.s32DeltIpQp,
                        &rc.h264.s32MaxReEncodeTimes,
                        &rc.h264.u32FrmMaxQp,
                        &rc.h264.u32FrmMinQp,
                        &rc.h264.u32FrmMinIQp,
                        &rc.h264.u32FrmMaxIQp,
                        &rc.h264.u32MotionStaticSwitchFrmQp,
                        &rc.h265.s32FirstFrameStartQp,
                        &rc.h265.u32StepQp,
                        &rc.h265.u32MinQp,
                        &rc.h265.u32MaxQp,
                        &rc.h265.u32MinIQp,
                        &rc.h265.u32MaxIQp,
                        &rc.h265.s32DeltIpQp,
                        &rc.h265.s32MaxReEncodeTimes,
                        &rc.h265.u32FrmMaxQp,
                        &rc.h265.u32FrmMinQp,
                        &rc.h265.u32FrmMinIQp,
                        &rc.h265.u32FrmMaxIQp,
                        &rc.h265.u32MotionStaticSwitchFrmQp);

                    if (parsed < 26)
                    {
                        write_json_error(ctrl_client_fd, seq, "invalid RC params");
                    }
                    else
                    {
                        video_set_rc(&rc);
                        write_json(ctrl_client_fd, "{seq: %d}", seq);
                    }
                }
                else if (strcmp("get_video_rc", method) == 0)
                {
                    VideoRcConfig rc;
                    video_get_rc(&rc);
                    write_json(ctrl_client_fd,
                        "{seq: %d, result: {"
                        "h264: {"
                        "s32FirstFrameStartQp: %d,"
                        "u32StepQp: %u,"
                        "u32MinQp: %u,"
                        "u32MaxQp: %u,"
                        "u32MinIQp: %u,"
                        "u32MaxIQp: %u,"
                        "s32DeltIpQp: %d,"
                        "s32MaxReEncodeTimes: %d,"
                        "u32FrmMaxQp: %u,"
                        "u32FrmMinQp: %u,"
                        "u32FrmMinIQp: %u,"
                        "u32FrmMaxIQp: %u,"
                        "u32MotionStaticSwitchFrmQp: %u"
                        "},"
                        "h265: {"
                        "s32FirstFrameStartQp: %d,"
                        "u32StepQp: %u,"
                        "u32MinQp: %u,"
                        "u32MaxQp: %u,"
                        "u32MinIQp: %u,"
                        "u32MaxIQp: %u,"
                        "s32DeltIpQp: %d,"
                        "s32MaxReEncodeTimes: %d,"
                        "u32FrmMaxQp: %u,"
                        "u32FrmMinQp: %u,"
                        "u32FrmMinIQp: %u,"
                        "u32FrmMaxIQp: %u,"
                        "u32MotionStaticSwitchFrmQp: %u"
                        "}"
                        "}}",
                        seq,
                        rc.h264.s32FirstFrameStartQp,
                        rc.h264.u32StepQp,
                        rc.h264.u32MinQp,
                        rc.h264.u32MaxQp,
                        rc.h264.u32MinIQp,
                        rc.h264.u32MaxIQp,
                        rc.h264.s32DeltIpQp,
                        rc.h264.s32MaxReEncodeTimes,
                        rc.h264.u32FrmMaxQp,
                        rc.h264.u32FrmMinQp,
                        rc.h264.u32FrmMinIQp,
                        rc.h264.u32FrmMaxIQp,
                        rc.h264.u32MotionStaticSwitchFrmQp,
                        rc.h265.s32FirstFrameStartQp,
                        rc.h265.u32StepQp,
                        rc.h265.u32MinQp,
                        rc.h265.u32MaxQp,
                        rc.h265.u32MinIQp,
                        rc.h265.u32MaxIQp,
                        rc.h265.s32DeltIpQp,
                        rc.h265.s32MaxReEncodeTimes,
                        rc.h265.u32FrmMaxQp,
                        rc.h265.u32FrmMinQp,
                        rc.h265.u32FrmMinIQp,
                        rc.h265.u32FrmMaxIQp,
                        rc.h265.u32MotionStaticSwitchFrmQp);
                }
                else if (strcmp("set_edid", method) == 0)
                {
                    uint8_t edid[256];
                    char *edid_hex;
                    if (json_scanf(buf_in, n, "{params: {edid: %Q} }", &edid_hex) > 0)
                    {
                        int edid_len = hex_to_bytes(edid_hex, edid, 256);
                        free(edid_hex);

                        if (edid_len < 0)
                        {
                            write_json_error(ctrl_client_fd, seq, "invalid edid");
                            continue;
                        }
                        if (set_edid(edid, edid_len) < 0)
                        {
                            write_json_error(ctrl_client_fd, seq, "failed to set EDID");
                        }
                        write_json(ctrl_client_fd, "{seq: %d}", seq);
                    }
                    else
                    {
                        write_json_error(ctrl_client_fd, seq, "invalid edid");
                    }
                }
                else if (strcmp("jpeg_capture_start", method) == 0)
                {
                    if (video_jpeg_capture(JPEG_CONTINUOUS) < 0)
                    {
                        write_json_error(ctrl_client_fd, seq, "capture start failed");
                    }
                    else
                    {
                        write_json(ctrl_client_fd, "{seq: %d}", seq);
                    }
                }
                else if (strcmp("jpeg_capture_stop", method) == 0)
                {
                    video_jpeg_stop();
                    write_json(ctrl_client_fd, "{seq: %d}", seq);
                }
                else if (strcmp("jpeg_take_snapshot", method) == 0)
                {
                    if (video_jpeg_get() < 0)
                    {
                        write_json_error(ctrl_client_fd, seq, "snapshot failed");
                    }
                    else
                    {
                        write_json(ctrl_client_fd, "{seq: %d}", seq);
                    }
                }
                else if (strcmp("get_edid", method) == 0)
                {
                    uint8_t edid[256];
                    int edid_len = get_edid(edid, 256);
                    if (edid_len < 0)
                    {
                        write_json_error(ctrl_client_fd, seq, "failed to get EDID");
                        continue;
                    }

                    char *edid_hex = bytes_to_hex(edid, edid_len);
                    if (edid_hex == NULL)
                    {
                        write_json_error(ctrl_client_fd, seq, "failed to convert EDID to hex");
                        free(edid_hex);
                        continue;
                    }

                    write_json(ctrl_client_fd, "{seq: %d, result: {edid: %Q}}", seq, edid_hex);
                    free(edid_hex);
                }
                free(method);
            }
        }
        else if (n == 0)
        {
            // EOF
            break;
        }
        else
        {
            perror("error read from ctrl client");
            break;
        }
    }

    close(ctrl_client_fd);
    return NULL;
}

int connect_ctrl_client(const char *path)
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
        perror("can not connect to ctrl server");
        close(client_fd);
        return -1;
    }
    ctrl_client_fd = client_fd;
    return 0;
}

pthread_t ctrl_thread;

void start_ctrl_loop()
{
    if (pthread_create(&ctrl_thread, NULL, (void *(*)(void *))handle_client, NULL) != 0)
    {
        perror("Failed to create ctrl_thread");
    }
}

void stop_ctrl_loop()
{
    if (ctrl_client_fd != 0)
    {
        if (shutdown(ctrl_client_fd, SHUT_RDWR) == -1)
        {
            perror("Error shutting down ctrl client socket");
        }
        if (close(ctrl_client_fd) == -1)
        {
            perror("Error closing client socket");
        }
        ctrl_client_fd = 0;
    }
}
