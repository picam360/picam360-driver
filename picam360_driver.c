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

#include "picam360_driver.h"
#include "MotionSensor.h"

#include "rtp.h"
#include "video_mjpeg.h"
#include "quaternion.h"

#include <mat4/multiply.h>
#include <mat4/transpose.h>
#include <mat4/fromQuat.h>
#include <mat4/invert.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define CONFIG_FILE "config.json"
#define PLUGIN_NAME "picam360_driver"

#define PT_STATUS 100
#define PT_CMD 101
int lg_cmd_fd = -1;

static bool lg_is_compass_calib = false;
static float lg_compass_min[3] = { -708.000000, -90.000000, -173.000000 };
//static float lg_compass_min[3] = { INT_MAX, INT_MAX, INT_MAX };
static float lg_compass_max[3] = { -47.000000, 536.000000, 486.000000 };
//static float lg_compass_max[3] = { -INT_MAX, -INT_MAX, -INT_MAX };
static VECTOR4D_T lg_quat = { };
static VECTOR4D_T lg_compass = { .ary = { 0, 0, 0, 1 } };
static float lg_north = 0;
static int lg_north_count = 0;

static int lg_i2c_ch = 1;

#define MOTOR_CENTER 0.0737
#define MOTOR_MERGIN 0.0013
#define MOTOR_RANGE 0.005
#define MOTOR_BASE(value) MOTOR_CENTER + MOTOR_MERGIN * ((value == 0) ? 0 : (value > 0) ? 1 : -1)

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static int lg_light_id[LIGHT_NUM] = { 4, 34 };
static int lg_motor_id[MOTOR_NUM] = { 18, 36, 35, 17 };

static int lg_light_value[LIGHT_NUM] = { 0, 0 };
static int lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_dir[4] = { -1, 1, -1, 1 };

#define MAX_DELAY_COUNT 256
static float lg_video_delay = 0;
static int lg_video_delay_cur = 0;
static VECTOR4D_T lg_quaternion_queue[MAX_DELAY_COUNT] = { };

static int lg_skip_frame = 0;
static int lg_ack_command_id = 0;

static void init_options();
static void save_options();

static void command_handler(const char *_buff) {
	char buff[256];
	strncpy(buff, _buff, sizeof(buff));
	char *cmd;
	cmd = strtok(buff, " \n");
	if (cmd == NULL) {
		//do nothing
	} else if (strncmp(cmd, PLUGIN_NAME ".start_compass_calib", sizeof(buff))
			== 0) {
		lg_is_compass_calib = true;
		for (int i = 0; i < 3; i++) {
			lg_compass_min[i] = INT_MAX;
			lg_compass_max[i] = -INT_MAX;
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".stop_compass_calib", sizeof(buff))
			== 0) {
		lg_is_compass_calib = false;
	} else if (strncmp(cmd, PLUGIN_NAME ".set_video_delay", sizeof(buff))
			== 0) {
		char *param = strtok(NULL, " \n");
		if (param != NULL) {
			float value = 0;
			sscanf(param, "%f", &value);
			lg_video_delay = MAX(MIN(value,MAX_DELAY_COUNT), 0);
			printf("set_video_delay : completed\n");
		}
	} else if (strncmp(cmd, PLUGIN_NAME ".save", sizeof(buff)) == 0) {
		save_options();
	} else {
		printf(":unknown command : %s\n", buff);
	}
}

static bool init_pwm() {
	ms_open(lg_i2c_ch);
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd > 0) {
		char cmd[256];
		int len;
		len = sprintf(cmd, "%d=%f\n", lg_light_id[0], 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_light_id[1], 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[0], MOTOR_CENTER);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[1], MOTOR_CENTER);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[2], MOTOR_CENTER);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor_id[3], MOTOR_CENTER);
		write(fd, cmd, len);
		close(fd);
	}
	return true;
}

static int xmp(char *buff, int buff_len) {
	int xmp_len = 0;

	static VECTOR4D_T quat = { };
	{
		int cur = (lg_video_delay_cur - (int) lg_video_delay + MAX_DELAY_COUNT)
				% MAX_DELAY_COUNT;
		quat = lg_quaternion_queue[cur];
	}

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
			lg_compass.x, lg_compass.y, lg_compass.z);
	if (lg_is_compass_calib) {
		xmp_len += sprintf(buff + xmp_len,
				"<compass_min x=\"%f\" y=\"%f\" z=\"%f\" />", lg_compass_min[0],
				lg_compass_min[1], lg_compass_min[2]);
		xmp_len += sprintf(buff + xmp_len,
				"<compass_max x=\"%f\" y=\"%f\" z=\"%f\" />", lg_compass_max[0],
				lg_compass_max[1], lg_compass_max[2]);
	}
	xmp_len += sprintf(buff + xmp_len, "<temperature v=\"%f\" />", temp);
	xmp_len += sprintf(buff + xmp_len, "<bandwidth v=\"%f\" />",
			rtp_get_bandwidth());
	for (int i = 0; i < 2; i++) {
		xmp_len += sprintf(buff + xmp_len,
				"<video_info id=\"%d\" fps=\"%f\" frameskip=\"%d\" />", i,
				video_mjpeg_get_fps(i), video_mjpeg_get_frameskip(i));
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

static float sub_angle(float a, float b) {
	float v = a - b;
	v -= floor(v / 360) * 360;
	if (v > 180.0) {
		v -= 360.0;
	}
	return v;
}

static bool lg_debugdump = false;
static void *transmit_thread_func(void* arg) {
	int count = 0;
	int xmp_len = 0;
	int buff_size = RTP_MAXPAYLOADSIZE;
	char buff[RTP_MAXPAYLOADSIZE];
	while (1) {
		count++;
		ms_update();
		{ //compas : calibration
			float calib[3];
			float bias[3];
			float gain[3];
			for (int i = 0; i < 3; i++) {
				if (lg_is_compass_calib) {
					lg_compass_min[i] = MIN(lg_compass_min[i], compass[i]);
					lg_compass_max[i] = MAX(lg_compass_max[i], compass[i]);
				}
				bias[i] = (lg_compass_min[i] + lg_compass_max[i]) / 2;
				gain[i] = (lg_compass_max[i] - lg_compass_min[i]) / 2;
				calib[i] = (compass[i] - bias[i])
						/ (gain[i] == 0 ? 1 : gain[i]);
			}
			float norm = sqrt(
					calib[0] * calib[0] + calib[1] * calib[1]
							+ calib[2] * calib[2]);
			for (int i = 0; i < 3; i++) {
				calib[i] /= norm;
			}
			//convert from mpu coodinate to opengl coodinate
			lg_compass.ary[0] = calib[1];
			lg_compass.ary[1] = -calib[0];
			lg_compass.ary[2] = -calib[2];
			lg_compass.ary[3] = 1.0;
		}
		{ //quat : convert from mpu coodinate to opengl coodinate
			lg_quat.ary[0] = quaternion[1];	//x
			lg_quat.ary[1] = quaternion[3];	//y : swap y and z
			lg_quat.ary[2] = -quaternion[2];	//z : swap y and z
			lg_quat.ary[3] = quaternion[0];	//w
		}
		{ //north
			float north = 0;

			float matrix[16];
			mat4_fromQuat(matrix, lg_quat.ary);
			mat4_invert(matrix, matrix);

			float compass_mat[16] = { };
			memcpy(compass_mat, lg_compass.ary, sizeof(float) * 4);
			if (lg_debugdump) {
				printf("compass1 %f : %f, %f, %f\n", lg_north, compass_mat[0],
						compass_mat[1], compass_mat[2]);
			}

			mat4_transpose(compass_mat, compass_mat);
			mat4_multiply(compass_mat, compass_mat, matrix);
			mat4_transpose(compass_mat, compass_mat);

			if (lg_debugdump) {
				printf("compass2 %f : %f, %f, %f\n", lg_north, compass_mat[0],
						compass_mat[1], compass_mat[2]);
			}

			north = -atan2(compass_mat[0], -compass_mat[2]) * 180 / M_PI; // start from z axis

			lg_north = lg_north
					+ sub_angle(north, lg_north) / (lg_north_count + 1);
			lg_north_count++;
			if (lg_north_count > 100) {
				lg_north_count = 100;
			}
		}
		{ //calib
			float x, y, z;
			if (lg_debugdump) {
				quaternion_get_euler(lg_quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("original %f : %f, %f, %f\n", lg_north, x * 180 / M_PI,
						y * 180 / M_PI, z * 180 / M_PI);
			}
			lg_quat = quaternion_multiply(
					quaternion_get_from_y(-lg_north * M_PI / 180), lg_quat); // Rv=RvoRvRn
			if (lg_debugdump) {
				quaternion_get_euler(lg_quat, &y, &x, &z, EULER_SEQUENCE_YXZ);
				printf("north   %f : %f, %f, %f\n", lg_north, x * 180 / M_PI,
						y * 180 / M_PI, z * 180 / M_PI);
			}
		}
		{
			int cur = (lg_video_delay_cur + 1) % MAX_DELAY_COUNT;
			lg_quaternion_queue[cur] = lg_quat;
			lg_video_delay_cur++;
		}
		if ((count % 100) == 0) {
			xmp_len = xmp(buff, buff_size);
			rtp_sendpacket((unsigned char*) buff, xmp_len, PT_STATUS);
		}
		usleep(10 * 1000); //less than 100Hz
	}
	return NULL;
}

static void parse_xml(char *xml) {
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd < 0) {
		return;
	}
	char *value_str;

	value_str = strstr(xml, "light0_value=");
	if (value_str) {
		char cmd[256];
		float value;
		int len;
		sscanf(value_str, "light0_value=\"%f\"", &value);
		value = MIN(MAX(value, 0), 100);
		if (value != lg_light_value[0]) {
			lg_light_value[0] = value;

			value = pow(value / 100, 3);
			len = sprintf(cmd, "%d=%f\n", lg_light_id[0], value);
			write(fd, cmd, len);
		}
	}

	value_str = strstr(xml, "light1_value=");
	if (value_str) {
		char cmd[256];
		float value;
		int len;
		sscanf(value_str, "light1_value=\"%f\"", &value);
		value = MIN(MAX(value, 0), 100);
		if (value != lg_light_value[1]) {
			lg_light_value[1] = value;

			value = pow(value / 100, 3);
			len = sprintf(cmd, "%d=%f\n", lg_light_id[1], value);
			write(fd, cmd, len);
		}
	}

	value_str = strstr(xml, "motor0_value=");
	if (value_str) {
		char cmd[256];
		float value;
		int len;
		sscanf(value_str, "motor0_value=\"%f\"", &value);
		value = MIN(MAX(value, -100), 100);
		if (value != lg_motor_value[0]) {
			lg_motor_value[0] = value;

			value = lg_dir[0] * (value / 100) * MOTOR_RANGE
					+ MOTOR_BASE(lg_dir[0] * value);
			len = sprintf(cmd, "%d=%f\n", lg_motor_id[0], value);
			write(fd, cmd, len);
		}
	}

	value_str = strstr(xml, "motor1_value=");
	if (value_str) {
		char cmd[256];
		float value;
		int len;
		sscanf(value_str, "motor1_value=\"%f\"", &value);
		value = MIN(MAX(value, -100), 100);
		if (value != lg_motor_value[1]) {
			lg_motor_value[1] = value;

			value = lg_dir[1] * (value / 100) * MOTOR_RANGE
					+ MOTOR_BASE(lg_dir[1] * value);
			len = sprintf(cmd, "%d=%f\n", lg_motor_id[1], value);
			write(fd, cmd, len);
		}
	}

	value_str = strstr(xml, "motor2_value=");
	if (value_str) {
		char cmd[256];
		float value;
		int len;
		sscanf(value_str, "motor2_value=\"%f\"", &value);
		value = MIN(MAX(value, -100), 100);
		if (value != lg_motor_value[2]) {
			lg_motor_value[2] = value;

			value = lg_dir[2] * (value / 100) * MOTOR_RANGE
					+ MOTOR_BASE(lg_dir[2] * value);
			len = sprintf(cmd, "%d=%f\n", lg_motor_id[2], value);
			write(fd, cmd, len);
		}
	}

	value_str = strstr(xml, "motor3_value=");
	if (value_str) {
		char cmd[256];
		float value;
		int len;
		sscanf(value_str, "motor3_value=\"%f\"", &value);
		value = MIN(MAX(value, -100), 100);
		if (value != lg_motor_value[3]) {
			lg_motor_value[3] = value;

			value = lg_dir[3] * (value / 100) * MOTOR_RANGE
					+ MOTOR_BASE(lg_dir[3] * value);
			len = sprintf(cmd, "%d=%f\n", lg_motor_id[3], value);
			write(fd, cmd, len);
		}
	}

	value_str = strstr(xml, "command_id=");
	if (value_str) {
		int command_id;
		sscanf(value_str, "command_id=\"%d\"", &command_id);
		if (command_id != lg_ack_command_id) {
			lg_ack_command_id = command_id;

			value_str = strstr(xml, "command=");
			if (value_str) {
				char command[1024];
				sscanf(value_str, "command=\"%[^\"]\"", command);
				command_handler(command);
			}
		}
	}

	close(fd);
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
		lg_i2c_ch = json_number_value(
				json_object_get(options, PLUGIN_NAME ".i2c_ch"));
		for (int i = 0; i < 3; i++) {
			char buff[256];
			sprintf(buff, PLUGIN_NAME ".compass_min_%d", i);
			lg_compass_min[i] = json_number_value(
					json_object_get(options, buff));
			sprintf(buff, PLUGIN_NAME ".compass_max_%d", i);
			lg_compass_max[i] = json_number_value(
					json_object_get(options, buff));
		}

		for (int i = 0; i < LIGHT_NUM; i++) {
			char buff[256];
			sprintf(buff, PLUGIN_NAME ".light%d_id", i);
			int id = (int) json_number_value(json_object_get(options, buff));
			if (id != 0) {
				lg_light_id[i] = id;
			}
		}

		for (int i = 0; i < MOTOR_NUM; i++) {
			char buff[256];
			sprintf(buff, PLUGIN_NAME ".motor%d_id", i);
			int id = (int) json_number_value(json_object_get(options, buff));
			if (id != 0) {
				lg_motor_id[i] = id;
			}
		}

		json_decref(options);
	}
}

static void save_options() {
	json_t *options = json_object();

	json_object_set_new(options, "skip_frame", json_real(lg_skip_frame));
	json_object_set_new(options, "video_delay", json_real(lg_video_delay));
	for (int i = 0; i < 3; i++) {
		char buff[256];
		sprintf(buff, PLUGIN_NAME ".compass_min_%d", i);
		json_object_set_new(options, buff, json_real(lg_compass_min[i]));
		sprintf(buff, PLUGIN_NAME ".compass_max_%d", i);
		json_object_set_new(options, buff, json_real(lg_compass_max[i]));
	}

	json_dump_file(options, CONFIG_FILE, 0);

	json_decref(options);
}

int main(int argc, char *argv[]) {
	bool succeeded;

	//init options
	init_options();

	rtp_set_callback((RTP_CALLBACK) rtp_callback);

	init_rtp(9004, "192.168.4.2", 9002);

	set_video_mjpeg_xmp_callback(xmp);
	init_video_mjpeg(0, NULL);
	init_video_mjpeg(1, NULL);
	video_mjpeg_set_skip_frame(0, lg_skip_frame);
	video_mjpeg_set_skip_frame(1, lg_skip_frame);

	succeeded = init_pwm();
	if (!succeeded) {
		perror("An error happen in init_pwm().");
		exit(-1);
	}

	pthread_t transmit_thread;
	pthread_create(&transmit_thread, NULL, transmit_thread_func, (void*) NULL);

	pthread_t recieve_thread;
	pthread_create(&recieve_thread, NULL, recieve_thread_func, (void*) NULL);

	//wait exit command
	pthread_join(recieve_thread, NULL);

	return 0;
}
