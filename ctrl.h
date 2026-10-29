#ifndef VIDEO_DAEMON_CTRL_H
#define VIDEO_DAEMON_CTRL_H

#include <stdbool.h>
#include <sys/types.h>

int connect_ctrl_client(const char *path);

void start_ctrl_loop();

void stop_ctrl_loop();

void report_video_format(bool ready, const char *error, u_int16_t width, u_int16_t height, double frame_per_second);

void report_jpeg_ready(const char *path);

// report_jpeg_failed tells the control client that a pending snapshot will never
// produce a file, so it can abort instead of waiting out its own timeout.
void report_jpeg_failed(const char *reason);

#endif //VIDEO_DAEMON_CTRL_H
