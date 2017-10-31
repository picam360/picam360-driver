#pragma once

#include <stdbool.h>
#include <pthread.h>
#include "mrevent.h"

#include "picam360_driver_plugin.h"

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
} PICAM360DRIVER_T;
