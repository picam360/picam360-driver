#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void init_video_mjpeg(int cam_num, void *user_data);
void deinit_video_mjpeg(int cam_num);
float video_mjpeg_get_fps(int cam_num);
int video_mjpeg_get_frameskip(int cam_num);

typedef int (*VIDEO_MJPEG_XMP_CALLBACK)(char *buff, int buff_len);
void set_video_mjpeg_xmp_callback(VIDEO_MJPEG_XMP_CALLBACK callback);

#ifdef __cplusplus
}
#endif
