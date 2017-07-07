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

#include <editline/readline.h>
#include <editline/history.h>

#include "picam360_driver.h"
#include "MotionSensor.h"

#include "rtp.h"
#include "video_mjpeg.h"
#include "quaternion.h"
#include "manual_mpu.h"

//these plugin should be got out to shared object
#include "plugins/rov_driver/rov_driver.h"
#include "plugins/mpu9250/mpu9250.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define CAMERA_NUM 2

#define CONFIG_FILE "config.json"
#define PATH "./"
#define PICAM360_HISTORY_FILE ".picam360-driver-history"
#define PLUGIN_NAME "picam360_driver"

static volatile int terminate;
static PICAM360DRIVER_T _state, *state = &_state;

#define PT_STATUS 100
#define PT_CMD 101
int lg_cmd_fd = -1;

static VECTOR4D_T lg_camera_offset[CAMERA_NUM] = { };

#define MAX_DELAY_COUNT 256
static float lg_video_delay = 0;
static int lg_video_delay_cur = 0;
static VECTOR4D_T lg_quaternion_queue[MAX_DELAY_COUNT] = { };

static int lg_skip_frame = 0;
static int lg_ack_command_id = 0;

static void init_options();
static void save_options();

static int _command_handler(const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".set_video_delay", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value = 0;
			sscanf(param, "%f", &value);
			lg_video_delay = MAX(MIN(value,MAX_DELAY_COUNT), 0);
			printf("set_video_delay : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_x", sizeof(buff))
			== 0) {
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
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_y", sizeof(buff))
			== 0) {
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
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_offset_yaw", sizeof(buff))
			== 0) {
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
	} else if (strncmp(cmd, PLUGIN_NAME ".add_camera_horizon_r", sizeof(buff))
			== 0) {
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
	} else if (strncmp(cmd, PLUGIN_NAME ".save", sizeof(buff)) == 0) {
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
		int cur = (lg_video_delay_cur - (int) lg_video_delay + MAX_DELAY_COUNT)
				% MAX_DELAY_COUNT;
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
	xmp_len +=
			sprintf(buff + xmp_len,
					"<x:xmpmeta xmlns:x=\"adobe:ns:meta/\" x:xmptk=\"picam360-drive rev1\">");
	xmp_len +=
			sprintf(buff + xmp_len,
					"<rdf:RDF xmlns:rdf=\"http://www.w3.org/1999/02/22-rdf-syntax-ns#\">");
	xmp_len += sprintf(buff + xmp_len, "<rdf:Description rdf:about=\"\">");
	xmp_len += sprintf(buff + xmp_len,
			"<quaternion x=\"%f\" y=\"%f\" z=\"%f\" w=\"%f\" />", quat.x,
			quat.y, quat.z, quat.w);
	xmp_len += sprintf(buff + xmp_len, "<compass x=\"%f\" y=\"%f\" z=\"%f\" />",
			compass.x, compass.y, compass.z);
//	if (lg_is_compass_calib) {
//		xmp_len += sprintf(buff + xmp_len,
//				"<compass_min x=\"%f\" y=\"%f\" z=\"%f\" />", lg_compass_min[0],
//				lg_compass_min[1], lg_compass_min[2]);
//		xmp_len += sprintf(buff + xmp_len,
//				"<compass_max x=\"%f\" y=\"%f\" z=\"%f\" />", lg_compass_max[0],
//				lg_compass_max[1], lg_compass_max[2]);
//	}
	xmp_len += sprintf(buff + xmp_len, "<temperature v=\"%f\" />", state->plugin_host.get_temperature());
	xmp_len += sprintf(buff + xmp_len, "<bandwidth v=\"%f\" />",
			rtp_get_bandwidth());
	for (int i = 0; i < 2; i++) {
		xmp_len += sprintf(buff + xmp_len,
				"<video_info id=\"%d\" fps=\"%f\" frameskip=\"%d\" />", i,
				video_mjpeg_get_fps(i), video_mjpeg_get_frameskip(i));
	}
	if (cam_num >= 0 && cam_num < CAMERA_NUM) {
		xmp_len += sprintf(buff + xmp_len,
				"<offset x=\"%f\" y=\"%f\" yaw=\"%f\" horizon_r=\"%f\" />",
				lg_camera_offset[cam_num].x, lg_camera_offset[cam_num].y,
				lg_camera_offset[cam_num].z, lg_camera_offset[cam_num].w);
	}
	xmp_len += sprintf(buff + xmp_len, "<ack_command_id v=\"%d\" />",
			lg_ack_command_id);
	xmp_len += sprintf(buff + xmp_len, "</rdf:Description>");
	xmp_len += sprintf(buff + xmp_len, "</rdf:RDF>");
	xmp_len += sprintf(buff + xmp_len, "</x:xmpmeta>");
	xmp_len += sprintf(buff + xmp_len, "<?xpacket end=\"w\"?>");
	buff[xmp_len++] = '\0';
	buff[2] = ((xmp_len - 2) >> 8) & 0xFF; // size MSB
	buff[3] = (xmp_len - 2) & 0xFF; // size LSB

	return xmp_len;
}

static void *transmit_thread_func(void* arg) {
	int count = 0;
	int xmp_len = 0;
	int buff_size = RTP_MAXPAYLOADSIZE;
	char buff[RTP_MAXPAYLOADSIZE];
	while (1) {
		count++;
		{
			int cur = (lg_video_delay_cur + 1) % MAX_DELAY_COUNT;
			lg_quaternion_queue[cur] = state->plugin_host.get_quaternion();
			lg_video_delay_cur++;
		}
		if ((count % 100) == 0) {
			xmp_len = xmp(buff, buff_size, -1);
			rtp_sendpacket((unsigned char*) buff, xmp_len, PT_STATUS);
		}
		usleep(10 * 1000); //less than 100Hz
	}
	return NULL;
}
//
static void parse_xml(char *xml) {
//	int fd = open("/dev/pi-blaster", O_WRONLY);
//	if (fd < 0) {
//		return;
//	}
//	char *value_str;
//
//	value_str = strstr(xml, "light0_value=");
//	if (value_str) {
//		char cmd[256];
//		float value;
//		int len;
//		sscanf(value_str, "light0_value=\"%f\"", &value);
//		value = MIN(MAX(value, 0), 100);
//		if (value != lg_light_value[0]) {
//			lg_light_value[0] = value;
//
//			value = pow(value / 100, 3);
//			len = sprintf(cmd, "%d=%f\n", lg_light_id[0], value);
//			write(fd, cmd, len);
//		}
//	}
//
//	value_str = strstr(xml, "light1_value=");
//	if (value_str) {
//		char cmd[256];
//		float value;
//		int len;
//		sscanf(value_str, "light1_value=\"%f\"", &value);
//		value = MIN(MAX(value, 0), 100);
//		if (value != lg_light_value[1]) {
//			lg_light_value[1] = value;
//
//			value = pow(value / 100, 3);
//			len = sprintf(cmd, "%d=%f\n", lg_light_id[1], value);
//			write(fd, cmd, len);
//		}
//	}
//
//	value_str = strstr(xml, "motor0_value=");
//	if (value_str) {
//		char cmd[256];
//		float value;
//		int len;
//		sscanf(value_str, "motor0_value=\"%f\"", &value);
//		value = MIN(MAX(value, -100), 100);
//		if (value != lg_motor_value[0]) {
//			lg_motor_value[0] = value;
//
//			value = lg_motor_dir[0] * (value / 100) * lg_motor_range
//					+ MOTOR_BASE(lg_motor_dir[0] * value);
//			len = sprintf(cmd, "%d=%f\n", lg_motor_id[0], value);
//			write(fd, cmd, len);
//		}
//	}
//
//	value_str = strstr(xml, "motor1_value=");
//	if (value_str) {
//		char cmd[256];
//		float value;
//		int len;
//		sscanf(value_str, "motor1_value=\"%f\"", &value);
//		value = MIN(MAX(value, -100), 100);
//		if (value != lg_motor_value[1]) {
//			lg_motor_value[1] = value;
//
//			value = lg_motor_dir[1] * (value / 100) * lg_motor_range
//					+ MOTOR_BASE(lg_motor_dir[1] * value);
//			len = sprintf(cmd, "%d=%f\n", lg_motor_id[1], value);
//			write(fd, cmd, len);
//		}
//	}
//
//	value_str = strstr(xml, "motor2_value=");
//	if (value_str) {
//		char cmd[256];
//		float value;
//		int len;
//		sscanf(value_str, "motor2_value=\"%f\"", &value);
//		value = MIN(MAX(value, -100), 100);
//		if (value != lg_motor_value[2]) {
//			lg_motor_value[2] = value;
//
//			value = lg_motor_dir[2] * (value / 100) * lg_motor_range
//					+ MOTOR_BASE(lg_motor_dir[2] * value);
//			len = sprintf(cmd, "%d=%f\n", lg_motor_id[2], value);
//			write(fd, cmd, len);
//		}
//	}
//
//	value_str = strstr(xml, "motor3_value=");
//	if (value_str) {
//		char cmd[256];
//		float value;
//		int len;
//		sscanf(value_str, "motor3_value=\"%f\"", &value);
//		value = MIN(MAX(value, -100), 100);
//		if (value != lg_motor_value[3]) {
//			lg_motor_value[3] = value;
//
//			value = lg_motor_dir[3] * (value / 100) * lg_motor_range
//					+ MOTOR_BASE(lg_motor_dir[3] * value);
//			len = sprintf(cmd, "%d=%f\n", lg_motor_id[3], value);
//			write(fd, cmd, len);
//		}
//	}
//
//	value_str = strstr(xml, "command_id=");
//	if (value_str) {
//		int command_id;
//		sscanf(value_str, "command_id=\"%d\"", &command_id);
//		if (command_id != lg_ack_command_id) {
//			lg_ack_command_id = command_id;
//
//			value_str = strstr(xml, "command=");
//			if (value_str) {
//				char command[1024];
//				sscanf(value_str, "command=\"%[^\"]\"", command);
//				_command_handler(command);
//			}
//		}
//	}
//
//	close(fd);
}

static void *recieve_thread_func(void* arg) {
	int buff_size = 4096;
	unsigned char *buff = malloc(buff_size);
	int data_len = 0;
	int marker = 0;
	int fd = open("cmd", O_RDONLY);
	if (fd < 0) {
		return NULL;
	}
	bool xmp = false;
	int buff_xmp_size = 4096;
	char *buff_xmp = malloc(buff_xmp_size);
	int xmp_len = 0;
	int xmp_idx = 0;

	while (1) {
		data_len = read(fd, buff, buff_size);
		for (int i = 0; i < data_len; i++) {
			if (xmp) {
				if (xmp_idx == 0) {
					xmp_len = ((unsigned char*) buff)[i] << 8;
				} else if (xmp_idx == 1) {
					xmp_len += ((unsigned char*) buff)[i];
					if (xmp_len > buff_xmp_size) {
						free(buff_xmp);
						buff_xmp_size = xmp_len;
						buff_xmp = malloc(buff_xmp_size);
					}
					buff_xmp[0] = (xmp_len >> 8) & 0xFF;
					buff_xmp[1] = (xmp_len) & 0xFF;
				} else {
					buff_xmp[xmp_idx] = buff[i];
				}
				xmp_idx++;
				if (xmp_idx >= xmp_len) {
					char *xml = buff_xmp + strlen(buff_xmp) + 1;

					parse_xml(xml);
					xmp = false;
				}
			}
			if (marker) {
				marker = 0;
				if (buff[i] == 0xE1) { //APP1
					xmp = true;
					xmp_len = 0;
					xmp_idx = 0;
				}
			} else if (buff[i] == 0xFF) {
				marker = 1;
			}
		}
	}

	free(buff);
	free(buff_xmp);

	return NULL;
}

static int rtp_callback(unsigned char *data, int data_len, int pt,
		unsigned int seq_num) {
	if (data_len <= 0) {
		return -1;
	}
	int fd = -1;
	if (pt == PT_CMD) {
		if (lg_cmd_fd < 0) {
			lg_cmd_fd = open("cmd", O_WRONLY | O_NONBLOCK);
		}
		fd = lg_cmd_fd;
	}
	if (fd < 0) {
		return -1;
	}
	write(fd, data, data_len);
	return 0;
}

static void init_options() {
	json_error_t error;
	json_t *options = json_load_file(CONFIG_FILE, 0, &error);
	if (options == NULL) {
		fputs(error.text, stderr);
	} else {
		lg_skip_frame = json_number_value(
				json_object_get(options, PLUGIN_NAME ".skip_frame"));
		lg_video_delay = json_number_value(
				json_object_get(options, PLUGIN_NAME ".video_delay"));

		for (int i = 0; i < CAMERA_NUM; i++) {
			char buff[256];
			sprintf(buff, PLUGIN_NAME ".cam%d_offset_x", i);
			lg_camera_offset[i].x = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, PLUGIN_NAME ".cam%d_offset_y", i);
			lg_camera_offset[i].y = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, PLUGIN_NAME ".cam%d_offset_yaw", i);
			lg_camera_offset[i].z = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, PLUGIN_NAME ".cam%d_horizon_r", i);
			lg_camera_offset[i].w = json_number_value(
					json_object_get(options, buff));
			if (lg_camera_offset[i].w == 0) {
				lg_camera_offset[i].w = 0.8;
			}
		}

		if (state->plugins) {
			for (int i = 0; state->plugins[i] != NULL; i++) {
				if (state->plugins[i]->init_options) {
					state->plugins[i]->init_options(
							state->plugins[i]->user_data, options);
				}
			}
		}

		json_decref(options);
	}
}

static void save_options() {
	json_t *options = json_object();

	json_object_set_new(options, PLUGIN_NAME ".skip_frame",
			json_real(lg_skip_frame));
	json_object_set_new(options, PLUGIN_NAME ".video_delay",
			json_real(lg_video_delay));

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

	if (state->plugins) {
		for (int i = 0; state->plugins[i] != NULL; i++) {
			if (state->plugins[i]->save_options) {
				state->plugins[i]->save_options(state->plugins[i]->user_data,
						options);
			}
		}
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
	for (int i = 0; state->plugins[i] != NULL; i++) {
		if (state->plugins[i]) {
			state->plugins[i]->event_handler(state->plugins[i]->user_data,
					node_id, event_id);
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

static void init_plugins(PICAM360DRIVER_T *state) {
	{ //init host
		state->plugin_host.get_quaternion = get_quaternion;
		state->plugin_host.get_compass = get_compass;
		state->plugin_host.get_temperature = get_temperature;
		state->plugin_host.get_north = get_north;

		state->plugin_host.send_command = send_command;
		state->plugin_host.send_event = send_event;
		state->plugin_host.add_mpu = add_mpu;
	}

	{
		MPU_T *mpu = NULL;
		create_manual_mpu(&mpu);
		state->plugin_host.add_mpu(mpu);
	}

	const int INITIAL_SPACE = 16;
	CREATE_PLUGIN create_plugin_funcs[] = { //
			create_rov_driver, //
					create_mpu9250, //
			};
	int num_of_plugins = sizeof(create_plugin_funcs) / sizeof(CREATE_PLUGIN);
	state->plugins = malloc(sizeof(PLUGIN_T*) * INITIAL_SPACE);
	memset(state->plugins, 0, sizeof(PLUGIN_T*) * INITIAL_SPACE);
	for (int i = 0; i < num_of_plugins; i++) {
		create_plugin_funcs[i](&state->plugin_host, &state->plugins[i]);
		state->plugins[i]->node_id = i + 100;
	}
	state->plugins[INITIAL_SPACE - 1] = (void*) -1;
}

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
		for (int i = 0; state->plugins[i] != NULL; i++) {
			int name_len = strlen(state->plugins[i]->name);
			if (strncmp(buff, state->plugins[i]->name, name_len) == 0
					&& buff[name_len] == '.') {
				ret = state->plugins[i]->command_handler(
						state->plugins[i]->user_data, buff);
				handled = true;
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
	while (ptr = readline("picam360-driver>")) {
		add_history(ptr);
		state->plugin_host.send_command(ptr);
		free(ptr);

		write_history(PICAM360_HISTORY_FILE);
	}
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

	rtp_set_callback((RTP_CALLBACK) rtp_callback);

	init_rtp(9004, "192.168.4.2", 9002, 0);

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

	pthread_t recieve_thread;
	pthread_create(&recieve_thread, NULL, recieve_thread_func, (void*) NULL);

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
