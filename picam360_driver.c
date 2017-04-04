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

#include "picam360_driver.h"
#include "MotionSensor.h"

#include "rtp.h"
#include "video_mjpeg.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define PT_STATUS 100
#define PT_CMD 101
int lg_cmd_fd = -1;

#define MOTOR_CENTER 0.0737
#define MOTOR_MERGIN 0.0013
#define MOTOR_RANGE 0.005
#define MOTOR_BASE(value) MOTOR_CENTER + MOTOR_MERGIN * ((value == 0) ? 0 : (value > 0) ? 1 : -1)

static float lg_compass_min[3] = { -708.000000, -90.000000, -173.000000 };
//static float lg_compass_min[3] = { INT_MAX, INT_MAX, INT_MAX };
static float lg_compass_max[3] = { -47.000000, 536.000000, 486.000000 };
//static float lg_compass_max[3] = { -INT_MAX, -INT_MAX, -INT_MAX };
static float lg_compass[4] = { };
static float lg_quat[4];
static int lg_light0_id = 4;
static int lg_light1_id = 34;
static int lg_motor0_id = 18;
static int lg_motor1_id = 36;
static int lg_motor2_id = 35;
static int lg_motor3_id = 17;

#define LIGHT_NUM 2
#define MOTOR_NUM 4
static int lg_light_value[LIGHT_NUM] = { 0, 0 };
static int lg_motor_value[MOTOR_NUM] = { 0, 0, 0, 0 };
static float lg_dir[4] = { -1, 1, -1, 1 };

bool init_pwm() {
	ms_open();
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd > 0) {
		char cmd[256];
		int len;
		len = sprintf(cmd, "%d=%f\n", lg_light0_id, 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_light1_id, 0.0f);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor0_id, MOTOR_CENTER);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor1_id, MOTOR_CENTER);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor2_id, MOTOR_CENTER);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_motor3_id, MOTOR_CENTER);
		write(fd, cmd, len);
		close(fd);
	}
	return true;
}

int xmp(char *buff, int buff_len) {
	int xmp_len = 0;

	ms_update();
	{
		lg_quat[0] = quatanion[0];
		lg_quat[1] = quatanion[1];
		lg_quat[2] = quatanion[2];
		lg_quat[3] = quatanion[3];
	}

	{ //compas : calibration
		float calib[3];
		float bias[3];
		float gain[3];
		for (int i = 0; i < 3; i++) {
			lg_compass_min[i] = MIN(lg_compass_min[i], compass[i]);
			lg_compass_max[i] = MAX(lg_compass_max[i], compass[i]);
			bias[i] = (lg_compass_min[i] + lg_compass_max[i]) / 2;
			gain[i] = (lg_compass_max[i] - lg_compass_min[i]) / 2;
			calib[i] = (compass[i] - bias[i]) / (gain[i] == 0 ? 1 : gain[i]);
		}

		//fprintf(stderr, "%f,%f,%f:%f,%f,%f:%f,%f,%f\n",
		//compass[0],compass[1],compass[2],
		//lg_compass_min[0],lg_compass_min[1],lg_compass_min[2],
		//lg_compass_max[0],lg_compass_max[1],lg_compass_max[2]
		//);

		float norm = sqrt(
				calib[0] * calib[0] + calib[1] * calib[1]
						+ calib[2] * calib[2]);
		for (int i = 0; i < 3; i++) {
			calib[i] /= norm;
		}
		lg_compass[0] = calib[0];
		lg_compass[1] = calib[1];
		lg_compass[2] = calib[2];
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
			"<quaternion w=\"%f\" x=\"%f\" y=\"%f\" z=\"%f\" />", lg_quat[0],
			lg_quat[1], lg_quat[2], lg_quat[3]);
	xmp_len += sprintf(buff + xmp_len, "<compass x=\"%f\" y=\"%f\" z=\"%f\" />",
			lg_compass[0], lg_compass[1], lg_compass[2]);
	xmp_len += sprintf(buff + xmp_len, "<temperature v=\"%f\" />", temp);
	xmp_len += sprintf(buff + xmp_len, "<bandwidth v=\"%f\" />",
			rtp_get_bandwidth());
	for (int i = 0; i < 2; i++) {
		xmp_len += sprintf(buff + xmp_len,
				"<video_info id=\"%d\" fps=\"%f\" frameskip=\"%d\" />", i,
				video_mjpeg_get_fps(i), video_mjpeg_get_frameskip(i));
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

void *transmit_thread_func(void* arg) {
	int xmp_len = 0;
	int buff_size = RTP_MAXPAYLOADSIZE;
	char buff[RTP_MAXPAYLOADSIZE];
	while (1) {
		xmp_len = xmp(buff, buff_size);
		rtp_sendpacket((unsigned char*) buff, xmp_len, PT_STATUS);

		usleep(10 * 1000); //less than 100Hz
	}
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
			len = sprintf(cmd, "%d=%f\n", lg_light0_id, value);
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
			len = sprintf(cmd, "%d=%f\n", lg_light1_id, value);
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
			len = sprintf(cmd, "%d=%f\n", lg_motor0_id, value);
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
			len = sprintf(cmd, "%d=%f\n", lg_motor1_id, value);
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
			len = sprintf(cmd, "%d=%f\n", lg_motor2_id, value);
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
			len = sprintf(cmd, "%d=%f\n", lg_motor3_id, value);
			write(fd, cmd, len);
		}
	}

	close(fd);
}

void *recieve_thread_func(void* arg) {
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

static int rtp_callback(unsigned char *data, int data_len, int pt) {
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

int main(int argc, char *argv[]) {
	bool succeeded;

	rtp_set_callback((RTP_CALLBACK) rtp_callback);

	init_rtp(9004, "192.168.4.2", 9002);
	init_video_mjpeg(0);
	init_video_mjpeg(1);

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
