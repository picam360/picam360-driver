#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void init_video();
void deinit_video();
float video_get_fps(int cam_num);
int video_get_frameskip(int cam_num);

#ifdef __cplusplus
}
#endif
