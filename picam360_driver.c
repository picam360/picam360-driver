/**
 * picam360-driver @ picam360 project
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <math.h>
#include <limits.h>
#include <jansson.h> //json parser
#include <dlfcn.h>

#include <editline/readline.h>
#include <editline/history.h>

#include "picam360_driver.h"

#include "rtp.h"
#include "rtcp.h"
#include "video_mjpeg.h"
#include "quaternion.h"
#include "manual_mpu.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define CAMERA_NUM 2

#define CONFIG_FILE "config.json"
#define PATH "./"
#define PICAM360_HISTORY_FILE ".picam360_history"
#define PLUGIN_NAME "picam360_driver"

static volatile int terminate;
static PICAM360DRIVER_T _state, *state = &_state;

#define PT_STATUS 100
#define PT_CMD 101

static VECTOR4D_T lg_camera_offset[CAMERA_NUM] = { };

#define MAX_DELAY_COUNT 256
static float lg_video_delay = 0;
static int lg_video_delay_cur = 0;
static VECTOR4D_T lg_quaternion_queue[MAX_DELAY_COUNT] = { };

static int lg_skip_frame = 0;
static int lg_ack_command_id = 0;

static void init_options();
static void save_options();

static void set_v4l2_ctl(const char *name, const int value) {
	for (int i = 0; i < CAMERA_NUM; i++) {
		char cmd[256];
		sprintf(cmd, "v4l2-ctl --set-ctrl=%s=%d -d /dev/video%d", name, value, (i == 0) ? 0 : 2);
		system(cmd);
	}
}

static void add_v4l2_ctl(V4l2_CTL_T *ctl) {
	if (state->v4l2_ctls == NULL) {
		const int INITIAL_SPACE = 16;
		state->v4l2_ctls = malloc(sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		memset(state->v4l2_ctls, 0, sizeof(V4l2_CTL_T*) * INITIAL_SPACE);
		state->v4l2_ctls[INITIAL_SPACE - 1] = (void*) -1;
	}

	for (int i = 0; state->v4l2_ctls[i] != (void*) -1; i++) {
		if (state->v4l2_ctls[i] == NULL) {
			state->v4l2_ctls[i] = ctl;
			return;
		}
		if (state->v4l2_ctls[i + 1] == (void*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			V4l2_CTL_T **current = state->v4l2_ctls;
			state->v4l2_ctls = malloc(sizeof(V4l2_CTL_T*) * space);
			memcpy(state->v4l2_ctls, current, sizeof(V4l2_CTL_T*) * (i + 1));
			state->v4l2_ctls[space - 1] = (void*) -1;
			free(current);
		}
	}
}

static int _command_handler(const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, "exit", sizeof(buff)) == 0 || strncmp(cmd, "q", sizeof(buff)) == 0 || strncmp(cmd, "quit", sizeof(buff)) == 0) {
		printf("exit\n");
		exit(0);
	} else if (strncmp(cmd, PLUGIN_NAME ".set_video_delay", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value = 0;
			sscanf(param, "%f", &value);
			lg_video_delay = MAX(MIN(value,MAX_DELAY_COUNT), 0);
			printf("set_video_delay : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_x", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			sscanf(param, "%d=%f", &cam_num, &value);
			if (cam_num >= 0 && cam_num < CAMERA_NUM) {
				lg_camera_offset[cam_num].x += value;
			}
			printf("add_camera_offset_x : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_y", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			sscanf(param, "%d=%f", &cam_num, &value);
			if (cam_num >= 0 && cam_num < CAMERA_NUM) {
				lg_camera_offset[cam_num].y += value;
			}
			printf("add_camera_offset_y : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_yaw", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			sscanf(param, "%d=%f", &cam_num, &value);
			if (cam_num >= 0 && cam_num < CAMERA_NUM) {
				lg_camera_offset[cam_num].z += value;
			}
			printf("add_camera_offset_yaw : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_horizon_r", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			int cam_num = 0;
			float value = 0;
			if (param[0] == '*') {
				sscanf(param, "*=%f", &value);
				for (int i = 0; i < CAMERA_NUM; i++) {
					lg_camera_offset[i].w += value;
				}
			} else {
				sscanf(param, "%d=%f", &cam_num, &value);
				if (cam_num >= 0 && cam_num < CAMERA_NUM) {
					lg_camera_offset[cam_num].w += value;
				}
			}
			printf("add_camera_horizon_r : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_v4l2_ctl", sizeof(buff)) == 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			char name[256] = { };
			int value = 0;
			int num = sscanf(param, "%255[^=]=%d", name, &value);
			if (num == 2) {
				for (int i = 0; state->v4l2_ctls[i] != NULL; i++) {
					if (strcmp(state->v4l2_ctls[i]->name, name) == 0) {
						state->v4l2_ctls[i]->value += value;
						set_v4l2_ctl(state->v4l2_ctls[i]->name, state->v4l2_ctls[i]->value);
						break;
					}
				}
			}

			printf("add_v4l2_ctl : completed\n");
		}
	} else if (strncmp(cmd, "save", sizeof(buff)) == 0) {
		save_options();
		printf("save : completed\n");
	} else {
		printf(":unknown command : %s\n", buff);
	}
	return 0;
}

static int xmp(char *buff, int buff_len, int cam_num) {
	int xmp_len = 0;

	VECTOR4D_T quat = { };
	{
		int cur = (lg_video_delay_cur - (int) lg_video_delay + MAX_DELAY_COUNT) % MAX_DELAY_COUNT;
		quat = lg_quaternion_queue[cur];
	}
	VECTOR4D_T compass = state->plugin_host.get_compass();

	xmp_len = 0;
	buff[xmp_len++] = 0xFF;
	buff[xmp_len++] = 0xE1;
	buff[xmp_len++] = 0; // size MSB
	buff[xmp_len++] = 0; // size LSB
	xmp_len += sprintf(buff + xmp_len, "http://ns.adobe.com/xap/1.0/");
	buff[xmp_len++] = '\0';
	xmp_len += sprintf(buff + xmp_len, "<?xpacket begin=\"﻿");
	buff[xmp_len++] = 0xEF;
	buff[xmp_len++] = 0xBB;
	buff[xmp_len++] = 0xBF;
	xmp_len += sprintf(buff + xmp_len, "\" id=\"W5M0MpCehiHzreSzNTczkc9d\"?>");
	xmp_len += sprintf(buff + xmp_len, "<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"picam360-drive rev1\">");
	xmp_len += sprintf(buff + xmp_len, "<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
	xmp_len += sprintf(buff + xmp_len, "<rdf:Description rdf:about=\"\">");
	xmp_len += sprintf(buff + xmp_len, "<quaternion x=\"%f\" y=\"%f\" z=\"%f\" w=\"%f\" />", quat.x, quat.y, quat.z, quat.w);
	xmp_len += sprintf(buff + xmp_len, "<compass x=\"%f\" y=\"%f\" z=\"%f\" />", compass.x, compass.y, compass.z);
	xmp_len += sprintf(buff + xmp_len, "<temperature v=\"%f\" />", state->plugin_host.get_temperature());
	if (cam_num >= 0 && cam_num < CAMERA_NUM) {
		xmp_len += sprintf(buff + xmp_len, "<offset x=\"%f\" y=\"%f\" yaw=\"%f\" horizon_r=\"%f\" />", lg_camera_offset[cam_num].x, lg_camera_offset[cam_num].y, lg_camera_offset[cam_num].z,
				lg_camera_offset[cam_num].w);
	}
	xmp_len += sprintf(buff + xmp_len, "</rdf:Description>");
	xmp_len += sprintf(buff + xmp_len, "</rdf:RDF>");
	xmp_len += sprintf(buff + xmp_len, "</x:xmpmeta>");
	xmp_len += sprintf(buff + xmp_len, "<?xpacket end=\"w\"?>");
	buff[xmp_len++] = '\0';
	buff[2] = ((xmp_len - 2) >> 8) & 0xFF; // size MSB
	buff[3] = (xmp_len - 2) & 0xFF; // size LSB

	return xmp_len;
}

///////////////////////////////////////////////////////
#if (1) //rtp block

static void *transmit_thread_func(void* arg) {
	int count = 0;
	while (1) {
		count++;
		{
			int cur = (lg_video_delay_cur + 1) % MAX_DELAY_COUNT;
			lg_quaternion_queue[cur] = state->plugin_host.get_quaternion();
			lg_video_delay_cur++;
		}
		if ((count % 10) == 0 && state->statuses) { //less than 10Hz
			int cur = 0;
			char statuses[RTP_MAXPAYLOADSIZE];
			for (int i = 0; state->statuses[i]; i++) {
				char value[256] = { };
				state->statuses[i]->get_value(state->statuses[i]->user_data, value, sizeof(value));
				char buff[256];
				int len = snprintf(buff, sizeof(buff), "<picam360:status name=\"%s\" value=\"%s\" />", state->statuses[i]->name, value);
				if (cur != 0 && cur + len > RTP_MAXPAYLOADSIZE) {
					rtp_sendpacket((unsigned char*) statuses, cur, PT_STATUS);
					cur = 0;
				} else {
					strncpy(statuses + cur, buff, len);
					cur += len;
				}
			}
			if (cur != 0) {
				rtp_sendpacket((unsigned char*) statuses, cur, PT_STATUS);
			}
		}
		usleep(10 * 1000); //less than 100Hz
	}
	return NULL;
}

static int rtcp_callback(unsigned char *data, int data_len, int pt, unsigned int seq_num) {
	if (data_len <= 0) {
		return -1;
	}
	static unsigned int last_seq_num = 0;
	if (seq_num != last_seq_num + 1) {
		printf("packet lost : from %d to %d\n", last_seq_num, seq_num);
	}
	last_seq_num = seq_num;

	if (pt == PT_CMD) {
		int id;
		char value[256];
		int num = sscanf((char*) data, "<picam360:command id=\"%d\" value=\"%255[^\"]\" />", &id, value);
		if (num == 2 && id != lg_ack_command_id) {
			lg_ack_command_id = id;
			state->plugin_host.send_command(value);
		}
	}
	return 0;
}

#if (1) //status block

#define STATUS_VAR(name) lg_status_ ## name
#define STATUS_INIT(plugin_host, prefix, name) STATUS_VAR(name) = new_status(prefix #name); \
                                               (plugin_host)->add_status(STATUS_VAR(name));

static STATUS_T *STATUS_VAR(ack_command_id);
static STATUS_T *STATUS_VAR(quaternion);
static STATUS_T *STATUS_VAR(compass);
static STATUS_T *STATUS_VAR(temperature);
static STATUS_T *STATUS_VAR(bandwidth);
static STATUS_T *STATUS_VAR(cam_fps);
static STATUS_T *STATUS_VAR(cam_frameskip);

static void status_release(void *user_data) {
	free(user_data);
}
static void status_get_value(void *user_data, char *buff, int buff_len) {
	STATUS_T *status = (STATUS_T*) user_data;
	if (status == STATUS_VAR(ack_command_id)) {
		snprintf(buff, buff_len, "%d", lg_ack_command_id);
	} else if (status == STATUS_VAR(quaternion)) {
		VECTOR4D_T vec = state->plugin_host.get_quaternion();
		snprintf(buff, buff_len, "%f,%f,%f,%f", vec.x, vec.y, vec.z, vec.w);
	} else if (status == STATUS_VAR(compass)) {
		VECTOR4D_T vec = state->plugin_host.get_compass();
		snprintf(buff, buff_len, "%f,%f,%f,%f", vec.x, vec.y, vec.z, vec.w);
	} else if (status == STATUS_VAR(temperature)) {
		snprintf(buff, buff_len, "%f", state->plugin_host.get_temperature());
	} else if (status == STATUS_VAR(bandwidth)) {
		snprintf(buff, buff_len, "%f", rtp_get_bandwidth());
	} else if (status == STATUS_VAR(cam_fps)) {
		snprintf(buff, buff_len, "%f,%f", video_mjpeg_get_fps(0), video_mjpeg_get_fps(1));
	} else if (status == STATUS_VAR(cam_frameskip)) {
		snprintf(buff, buff_len, "%d,%d", video_mjpeg_get_frameskip(0), video_mjpeg_get_frameskip(1));
	}
}

static void status_set_value(void *user_data, const char *value) {
	//STATUS_T *status = (STATUS_T*) user_data;
}

static STATUS_T *new_status(const char *name) {
	STATUS_T *status = (STATUS_T*) malloc(sizeof(STATUS_T));
	strcpy(status->name, name);
	status->get_value = status_get_value;
	status->set_value = status_set_value;
	status->release = status_release;
	status->user_data = status;
	return status;
}

static void init_status() {
	STATUS_INIT(&state->plugin_host, "", ack_command_id);
	STATUS_INIT(&state->plugin_host, "", quaternion);
	STATUS_INIT(&state->plugin_host, "", compass);
	STATUS_INIT(&state->plugin_host, "", temperature);
	STATUS_INIT(&state->plugin_host, "", bandwidth);
	STATUS_INIT(&state->plugin_host, "", cam_fps);
	STATUS_INIT(&state->plugin_host, "", cam_frameskip);
}

#endif //status block

static void _init_rtp() {
	init_rtp(9004, "192.168.4.2", 9002, 0);
	//rtp_set_callback((RTP_CALLBACK) rtp_callback);
	init_rtcp(9005, "192.168.4.2", 9003, 0);
	rtcp_set_callback((RTCP_CALLBACK) rtcp_callback);

	init_status();
}

#endif //rtp block

///////////////////////////////////////////
#if (1) //plugin host methods

static void init_options() {
	json_error_t error;
	json_t *options = json_load_file(CONFIG_FILE, 0, &error);
	if (options == NULL) {
		fputs(error.text, stderr);
	} else {
		lg_skip_frame = json_number_value(json_object_get(options, PLUGIN_NAME ".skip_frame"));
		lg_video_delay = json_number_value(json_object_get(options, PLUGIN_NAME ".video_delay"));

		for (int i = 0; i < CAMERA_NUM; i++) {
			char buff[256];
			sprintf(buff, PLUGIN_NAME ".cam%d_offset_x", i);
			lg_camera_offset[i].x = json_number_value(json_object_get(options, buff));
			sprintf(buff, PLUGIN_NAME ".cam%d_offset_y", i);
			lg_camera_offset[i].y = json_number_value(json_object_get(options, buff));
			sprintf(buff, PLUGIN_NAME ".cam%d_offset_yaw", i);
			lg_camera_offset[i].z = json_number_value(json_object_get(options, buff));
			sprintf(buff, PLUGIN_NAME ".cam%d_horizon_r", i);
			lg_camera_offset[i].w = json_number_value(json_object_get(options, buff));
			if (lg_camera_offset[i].w == 0) {
				lg_camera_offset[i].w = 0.8;
			}
		}
		{
			json_t *plugin_paths = json_object_get(options, "plugin_paths");
			if (json_is_array(plugin_paths)) {
				int size = json_array_size(plugin_paths);
				state->plugin_paths = (char**) malloc(sizeof(char*) * (size + 1));
				memset(state->plugin_paths, 0, sizeof(char*) * (size + 1));

				for (int i = 0; i < size; i++) {
					json_t *value = json_array_get(plugin_paths, i);
					int len = json_string_length(value);
					state->plugin_paths[i] = (char*) malloc(sizeof(char) * (len + 1));
					memset(state->plugin_paths[i], 0, sizeof(char) * (len + 1));
					strncpy(state->plugin_paths[i], json_string_value(value), len);
					if (len > 0) {
						void *handle = dlopen(state->plugin_paths[i], RTLD_LAZY);
						if (!handle) {
							fprintf(stderr, "%s\n", dlerror());
							continue;
						}
						CREATE_PLUGIN create_plugin = (CREATE_PLUGIN) dlsym(handle, "create_plugin");
						if (!create_plugin) {
							fprintf(stderr, "%s\n", dlerror());
							dlclose(handle);
							continue;
						}
						PLUGIN_T *plugin = NULL;
						create_plugin(&state->plugin_host, &plugin);
						if (!plugin) {
							fprintf(stderr, "%s\n", "create_plugin fail.");
							dlclose(handle);
							continue;
						}
						state->plugin_host.add_plugin(plugin);
					}
				}
			}
		}
		{
			json_t *v4l2_ctls = json_object_get(options, "v4l2_ctls");
			const char *key;
			json_t *value;

			json_object_foreach(v4l2_ctls, key, value)
			{
				V4l2_CTL_T *ctl = (V4l2_CTL_T*) malloc(sizeof(V4l2_CTL_T));
				strncpy(ctl->name, key, sizeof(ctl->name));
				ctl->value = json_number_value(value);
				add_v4l2_ctl(ctl);
			}
		}

		if (state->plugins) {
			for (int i = 0; state->plugins[i] != NULL; i++) {
				if (state->plugins[i]->init_options) {
					state->plugins[i]->init_options(state->plugins[i]->user_data, options);
				}
			}
		}

		if (state->v4l2_ctls) {
			for (int i = 0; state->v4l2_ctls[i] != NULL; i++) {
				set_v4l2_ctl(state->v4l2_ctls[i]->name, state->v4l2_ctls[i]->value);
			}
		}

		json_decref(options);
	}
}

static void save_options() {
	json_t *options = json_object();

	json_object_set_new(options, PLUGIN_NAME ".skip_frame", json_real(lg_skip_frame));
	json_object_set_new(options, PLUGIN_NAME ".video_delay", json_real(lg_video_delay));

	for (int i = 0; i < CAMERA_NUM; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".cam%d_offset_x", i);
		json_object_set_new(options, buff, json_real(lg_camera_offset[i].x));
		sprintf(buff, PLUGIN_NAME ".cam%d_offset_y", i);
		json_object_set_new(options, buff, json_real(lg_camera_offset[i].y));
		sprintf(buff, PLUGIN_NAME ".cam%d_offset_yaw", i);
		json_object_set_new(options, buff, json_real(lg_camera_offset[i].z));
		sprintf(buff, PLUGIN_NAME ".cam%d_horizon_r", i);
		json_object_set_new(options, buff, json_real(lg_camera_offset[i].w));
	}

	if (state->plugin_paths) {
		json_t *plugin_paths = json_array();
		for (int i = 0; state->plugin_paths[i] != NULL; i++) {
			json_array_append_new(plugin_paths, json_string(state->plugin_paths[i]));
		}
		json_object_set_new(options, "plugin_paths", plugin_paths);
	}

	if (state->plugins) {
		for (int i = 0; state->plugins[i] != NULL; i++) {
			if (state->plugins[i]->save_options) {
				state->plugins[i]->save_options(state->plugins[i]->user_data, options);
			}
		}
	}

	if (state->v4l2_ctls) {
		json_t *v4l2_ctls = json_object();
		for (int i = 0; state->v4l2_ctls[i] != NULL; i++) {
			json_object_set(v4l2_ctls, state->v4l2_ctls[i]->name, json_real(state->v4l2_ctls[i]->value));
		}
		json_object_set_new(options, "v4l2_ctls", v4l2_ctls);
	}

	json_dump_file(options, CONFIG_FILE, JSON_INDENT(4));

	json_decref(options);
}

//plugin host methods
static VECTOR4D_T get_quaternion() {
	VECTOR4D_T ret = { };
	if (state->mpu) {
		ret = state->mpu->get_quaternion(state->mpu);
	}
	return ret;
}
static VECTOR4D_T get_compass() {
	VECTOR4D_T ret = { };
	if (state->mpu) {
		ret = state->mpu->get_compass(state->mpu);
	}
	return ret;
}
static float get_temperature() {
	if (state->mpu) {
		return state->mpu->get_temperature(state->mpu);
	}
	return 0;
}
static float get_north() {
	if (state->mpu) {
		return state->mpu->get_north(state->mpu);
	}
	return 0;
}
static void send_command(const char *cmd) {
	pthread_mutex_lock(&state->cmd_list_mutex);

	LIST_T **cur = &state->cmd_list;
	for (; *cur != NULL; cur = &(*cur)->next)
		;
	*cur = malloc(sizeof(LIST_T));
	memset(*cur, 0, sizeof(LIST_T));
	int slr_len = MIN(strlen(cmd), 256);
	char *cmd_clone = malloc(slr_len + 1);
	strncpy(cmd_clone, cmd, slr_len);
	cmd_clone[slr_len] = '\0';
	(*cur)->value = cmd_clone;

	pthread_mutex_unlock(&state->cmd_list_mutex);
}
static void send_event(uint32_t node_id, uint32_t event_id) {
	if (state->plugins) {
		for (int i = 0; state->plugins[i] != NULL; i++) {
			if (state->plugins[i]) {
				state->plugins[i]->event_handler(state->plugins[i]->user_data, node_id, event_id);
			}
		}
	}
}
static void add_mpu(MPU_T *mpu) {
	if (state->mpus == NULL) {
		const int INITIAL_SPACE = 16;
		state->mpus = malloc(sizeof(MPU_T*) * INITIAL_SPACE);
		memset(state->mpus, 0, sizeof(MPU_T*) * INITIAL_SPACE);
		state->mpus[INITIAL_SPACE - 1] = (void*) -1;
	}

	for (int i = 0; state->mpus[i] != (void*) -1; i++) {
		if (state->mpus[i] == NULL) {
			state->mpus[i] = mpu;
			return;
		}
		if (state->mpus[i + 1] == (void*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			MPU_T **current = state->mpus;
			state->mpus = malloc(sizeof(MPU_T*) * space);
			memcpy(state->mpus, current, sizeof(MPU_T*) * (i + 1));
			state->mpus[space - 1] = (void*) -1;
			free(current);
		}
	}
}

static void add_status(STATUS_T *status) {
	if (state->statuses == NULL) {
		const int INITIAL_SPACE = 16;
		state->statuses = malloc(sizeof(STATUS_T*) * INITIAL_SPACE);
		memset(state->statuses, 0, sizeof(STATUS_T*) * INITIAL_SPACE);
		state->statuses[INITIAL_SPACE - 1] = (void*) -1;
	}

	for (int i = 0; state->statuses[i] != (void*) -1; i++) {
		if (state->statuses[i] == NULL) {
			state->statuses[i] = status;
			return;
		}
		if (state->statuses[i + 1] == (void*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			STATUS_T **current = state->statuses;
			state->statuses = malloc(sizeof(STATUS_T*) * space);
			memcpy(state->statuses, current, sizeof(STATUS_T*) * (i + 1));
			state->statuses[space - 1] = (void*) -1;
			free(current);
		}
	}
}

static void add_watch(STATUS_T *watch) {
	if (state->watches == NULL) {
		const int INITIAL_SPACE = 16;
		state->watches = malloc(sizeof(STATUS_T*) * INITIAL_SPACE);
		memset(state->watches, 0, sizeof(STATUS_T*) * INITIAL_SPACE);
		state->watches[INITIAL_SPACE - 1] = (void*) -1;
	}

	for (int i = 0; state->watches[i] != (void*) -1; i++) {
		if (state->watches[i] == NULL) {
			state->watches[i] = watch;
			return;
		}
		if (state->watches[i + 1] == (void*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			STATUS_T **current = state->watches;
			state->watches = malloc(sizeof(STATUS_T*) * space);
			memcpy(state->watches, current, sizeof(STATUS_T*) * (i + 1));
			state->watches[space - 1] = (void*) -1;
			free(current);
		}
	}
}

static void add_plugin(PLUGIN_T *plugin) {
	if (state->plugins == NULL) {
		const int INITIAL_SPACE = 16;
		state->plugins = malloc(sizeof(PLUGIN_T*) * INITIAL_SPACE);
		memset(state->plugins, 0, sizeof(PLUGIN_T*) * INITIAL_SPACE);
		state->plugins[INITIAL_SPACE - 1] = (void*) -1;
	}

	for (int i = 0; state->plugins[i] != (void*) -1; i++) {
		if (state->plugins[i] == NULL) {
			state->plugins[i] = plugin;
			return;
		}
		if (state->plugins[i + 1] == (void*) -1) {
			int space = (i + 2) * 2;
			if (space > 256) {
				fprintf(stderr, "error on add_mpu\n");
				return;
			}
			PLUGIN_T **current = state->plugins;
			state->plugins = malloc(sizeof(PLUGIN_T*) * space);
			memcpy(state->plugins, current, sizeof(PLUGIN_T*) * (i + 1));
			state->plugins[space - 1] = (void*) -1;
			free(current);
		}
	}
}

static void init_plugins(PICAM360DRIVER_T *state) {
	{ //init host
		state->plugin_host.get_quaternion = get_quaternion;
		state->plugin_host.get_compass = get_compass;
		state->plugin_host.get_temperature = get_temperature;
		state->plugin_host.get_north = get_north;

		state->plugin_host.send_command = send_command;
		state->plugin_host.send_event = send_event;
		state->plugin_host.add_mpu = add_mpu;
		state->plugin_host.add_status = add_status;
		state->plugin_host.add_watch = add_watch;
		state->plugin_host.add_plugin = add_plugin;
	}

	{
		MPU_T *mpu = NULL;
		create_manual_mpu(&mpu);
		state->plugin_host.add_mpu(mpu);
	}
}

#endif //plugin block

int command_handler() {
	int ret = 0;
	char *buff = NULL;

	{
		pthread_mutex_lock(&state->cmd_list_mutex);

		if (state->cmd_list) {
			LIST_T *cur = state->cmd_list;
			buff = (char*) cur->value;
			state->cmd_list = cur->next;
			free(cur);
		}

		pthread_mutex_unlock(&state->cmd_list_mutex);
	}

	if (buff) {
		bool handled = false;
		if (state->plugins) {
			for (int i = 0; state->plugins[i] != NULL; i++) {
				int name_len = strlen(state->plugins[i]->name);
				if (strncmp(buff, state->plugins[i]->name, name_len) == 0 && buff[name_len] == '.') {
					ret = state->plugins[i]->command_handler(state->plugins[i]->user_data, buff);
					handled = true;
				}
			}
		}
		if (!handled) {
			ret = _command_handler(buff);
		}
		free(buff);
	}
	return ret;
}

static void *readline_thread_func(void* arg) {
	char *ptr;
	using_history();
	read_history(PICAM360_HISTORY_FILE);
	while ((ptr = readline("picam360-driver>")) != NULL) {
		add_history(ptr);
		state->plugin_host.send_command(ptr);
		free(ptr);

		write_history(PICAM360_HISTORY_FILE);
	}
	return NULL;
}

//------------------------------------------------------------------------------

static void exit_func(void) {
	printf("\npicam360-driver closed\n");
} // exit_func()

int main(int argc, char *argv[]) {
	int opt;

// Clear application state
	memset(state, 0, sizeof(*state));
	strncpy(state->mpu_type, "manual", 64);

	umask(0000);

	while ((opt = getopt(argc, argv, "v:")) != -1) {
		switch (opt) {
		case 'v':
			strncpy(state->mpu_type, optarg, 64);
			break;
		default:
			/* '?' */
			printf("Usage: %s [-v mpu_type]\n", argv[0]);
			return -1;
		}
	}

	{ //mrevent & mutex init
		pthread_mutex_init(&state->mutex, 0);
		pthread_mutex_init(&state->cmd_list_mutex, 0);
	}

// init plugin
	init_plugins(state);

//init options
	init_options();

//init rtp
	_init_rtp();

//set mpu
	for (int i = 0; state->mpus[i] != NULL; i++) {
		if (strncmp(state->mpus[i]->name, state->mpu_type, 64) == 0) {
			state->mpu = state->mpus[i];
		}
	}

	set_video_mjpeg_xmp_callback(xmp);
	init_video_mjpeg(0, NULL);
	init_video_mjpeg(1, NULL);
	video_mjpeg_set_skip_frame(0, lg_skip_frame);
	video_mjpeg_set_skip_frame(1, lg_skip_frame);

	pthread_t transmit_thread;
	pthread_create(&transmit_thread, NULL, transmit_thread_func, (void*) NULL);

//readline
	pthread_t readline_thread;
	pthread_create(&readline_thread, NULL, readline_thread_func, (void*) NULL);

	while (!terminate) {
		command_handler();
		usleep(1000);
	}
	exit_func();

	return 0;
}
