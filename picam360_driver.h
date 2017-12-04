#pragma once

#include <stdbool.h>
#include <pthread.h>
#include "mrevent.h"

#include "picam360_driver_plugin.h"
#include "video_mjpeg.h"

#define CAMERA_NUM 2

typedef struct _OPTIONS_T {
	enum VIDEO_STREAM_TYPE vstream_type[CAMERA_NUM];
	char v4l2_devicefile[CAMERA_NUM][256];
	int cam_width[CAMERA_NUM];
	int cam_height[CAMERA_NUM];
	int cam_fps[CAMERA_NUM];
} OPTIONS_T;

typedef struct _LIST_T {
	void *value;
	struct _LIST_T *next;
} LIST_T;

typedef struct _V4l2_CTL_T {
	char name[256];
	int value;
} V4l2_CTL_T;

typedef struct _PICAM360DRIVER_T {
	PLUGIN_HOST_T plugin_host;
	pthread_mutex_t mutex;

	pthread_mutex_t cmd_list_mutex;
	LIST_T *cmd_list;

	char **plugin_paths;
	PLUGIN_T **plugins;
	MPU_T **mpus;
	MPU_T *mpu;
	STATUS_T **statuses;
	STATUS_T **watches;
	char mpu_type[64];
	V4l2_CTL_T **v4l2_ctls;
	OPTIONS_T options;
} PICAM360DRIVER_T;
