#pragma once

#include <stdint.h>
#include <pthread.h>
#include <wchar.h>
#include <jansson.h>//json parser
#include "quaternion.h"

#define PICAM360_HOST_NODE_ID 0

typedef struct _MPU_T {
	char name[64];
	void (*release)(void *user_data);
	VECTOR4D_T (*get_quaternion)(void *user_data);
	VECTOR4D_T (*get_compass)(void *user_data);
	float (*get_temperature)(void *user_data);
	float (*get_north)(void *user_data);
	void *user_data;
} MPU_T;

typedef struct _STATUS_T {
	char name[64];
	void (*get_value)(void *user_data, char *buff, int buff_len);
	void (*set_value)(void *user_data, const char *value);
	void (*release)(void *user_data);
	void *user_data;
} STATUS_T;

typedef struct _PLUGIN_T {
	char name[64];
	void (*release)(void *user_data);
	int (*command_handler)(void *user_data, const char *cmd);
	void (*event_handler)(void *user_data, uint32_t node_id, uint32_t event_id);
	void (*init_options)(void *user_data, json_t *options);
	void (*save_options)(void *user_data, json_t *options);
	wchar_t *(*get_info)(void *user_data);
	void *user_data;
	uint32_t node_id;
} PLUGIN_T;

typedef struct _PLUGIN_HOST_T {
	VECTOR4D_T (*get_quaternion)();
	VECTOR4D_T (*get_compass)();
	float (*get_temperature)();
	float (*get_north)();

	void (*send_command)(const char *cmd);
	void (*send_event)(uint32_t node_id, uint32_t event_id);
	void (*add_mpu)(MPU_T *mpu);
	void (*add_status)(STATUS_T *status);
	void (*add_watch)(STATUS_T *status);
	void (*add_plugin)(PLUGIN_T *plugin);
} PLUGIN_HOST_T;

typedef void (*CREATE_PLUGIN)(PLUGIN_HOST_T *plugin_host, PLUGIN_T **plugin);

