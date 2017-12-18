#pragma once

#include <stdbool.h>
#include <pthread.h>
#include "mrevent.h"
#include "rtp.h"

#include "picam360_driver_plugin.h"
#include "video_mjpeg.h"

#define CAMERA_NUM 2

typedef struct _OPTIONS_T {
	enum VIDEO_STREAM_TYPE vstream_type[CAMERA_NUM];
	char v4l2_devicefile[CAMERA_NUM][256];
	int cam_width[CAMERA_NUM];
	int cam_height[CAMERA_NUM];
	int cam_fps[CAMERA_NUM];

	int rtp_rx_port;
	enum RTP_SOCKET_TYPE rtp_rx_type;
	char rtp_tx_ip[256];
	int rtp_tx_port;
	enum RTP_SOCKET_TYPE rtp_tx_type;

	int rtcp_rx_port;
	enum RTP_SOCKET_TYPE rtcp_rx_type;
	char rtcp_tx_ip[256];
	int rtcp_tx_port;
	enum RTP_SOCKET_TYPE rtcp_tx_type;
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

	RTP_T *rtp;
	RTP_T *rtcp;

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
