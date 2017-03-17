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

#include "picam360_driver.h"
#include "MotionSensor.h"

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define MOTOR_CENTER 0.0744
#define MOTOR_RANGE 0.005

static float lg_quat[4];
static int lg_light0_id = 4;
static int lg_light1_id = 34;
static int lg_motor0_id = 17;
static int lg_motor1_id = 18;
static int lg_motor2_id = 35;
static int lg_motor3_id = 36;

bool init() {
	ms_open();
	int fd = open("/dev/pi-blaster", O_WRONLY);
	if (fd > 0) {
		char cmd[256];
		int len;
		len = sprintf(cmd, "%d=%f\n", lg_light0_id, 0);
		write(fd, cmd, len);
		len = sprintf(cmd, "%d=%f\n", lg_light1_id, 0);
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

float *get_quatanion_mpu9250() {
	ms_update();
	lg_quat[0] = quatanion[0];
	lg_quat[1] = quatanion[1];
	lg_quat[2] = quatanion[2];
	lg_quat[3] = quatanion[3];
	return lg_quat;
}

int xmp(char *buff, int buff_len) {
	int xmp_len = 0;

	float *quat = get_quatanion_mpu9250();

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
			"<quaternion w=\"%f\" x=\"%f\" y=\"%f\" z=\"%f\" />", quat[0],
			quat[1], quat[2], quat[3]);
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
	bool succeeded;

	succeeded = init();
	if (!succeeded) {
		perror("An error happen in init().");
		exit(-1);
	}
	int xmp_len = 0;
	int buff_size = 4096;
	char buff[buff_size];
	while (1) {
		xmp_len = xmp(buff, buff_size);
		write(STDOUT_FILENO, buff, xmp_len);

		usleep(10 * 1000); //less than 100Hz
	}
}

void *recieve_thread_func(void* arg) {
	int buff_size = 4096;
	unsigned char *buff = malloc(buff_size);
	int data_len = 0;
	int marker = 0;
	int file_fd = STDIN_FILENO;
	bool xmp = false;
	char *buff_xmp = NULL;
	int xmp_len = 0;
	int xmp_idx = 0;

	while (1) {
		data_len = read(file_fd, buff, buff_size);
		for (int i = 0; i < data_len; i++) {
			if (xmp) {
				if (xmp_idx == 0) {
					xmp_len = ((unsigned char*) buff)[i] << 8;
				} else if (xmp_idx == 1) {
					xmp_len += ((unsigned char*) buff)[i];
					buff_xmp = malloc(xmp_len);
					buff_xmp[0] = (xmp_len >> 8) & 0xFF;
					buff_xmp[1] = (xmp_len) & 0xFF;
				} else {
					buff_xmp[xmp_idx] = buff[i];
				}
				xmp_idx++;
				if (xmp_idx >= xmp_len) {
					int fd = open("/dev/pi-blaster", O_WRONLY);
					if (fd > 0) {
						char *xml = buff_xmp + strlen(buff_xmp) + 1;

						char *value_str;

						value_str = strstr(xml, "light0_value=");
						if (value_str) {
							char cmd[256];
							float value;
							int len;
							sscanf(value_str, "light0_value=\"%f\"", &value);
							value = MIN(MAX(value, 0), 100);
							value = pow(value / 100, 3);
							len = sprintf(cmd, "%d=%f\n", lg_light0_id, value);
							write(fd, cmd, len);
						}

						value_str = strstr(xml, "light1_value=");
						if (value_str) {
							char cmd[256];
							float value;
							int len;
							sscanf(value_str, "light1_value=\"%f\"", &value);
							value = MIN(MAX(value, 0), 100);
							value = pow(value / 100, 3);
							len = sprintf(cmd, "%d=%f\n", lg_light1_id, value);
							write(fd, cmd, len);
						}

						value_str = strstr(xml, "motor0_value=");
						if (value_str) {
							char cmd[256];
							float value;
							int len;
							sscanf(value_str, "motor0_value=\"%f\"", &value);
							value = MIN(MAX(value, -100), 100);
							value = (value / 100) * MOTOR_RANGE + MOTOR_CENTER;
							len = sprintf(cmd, "%d=%f\n", lg_motor0_id, value);
							write(fd, cmd, len);
						}

						value_str = strstr(xml, "motor1_value=");
						if (value_str) {
							char cmd[256];
							float value;
							int len;
							sscanf(value_str, "motor1_value=\"%f\"", &value);
							value = MIN(MAX(value, -100), 100);
							value = (value / 100) * MOTOR_RANGE + MOTOR_CENTER;
							len = sprintf(cmd, "%d=%f\n", lg_motor1_id, value);
							write(fd, cmd, len);
						}

						value_str = strstr(xml, "motor2_value=");
						if (value_str) {
							char cmd[256];
							float value;
							int len;
							sscanf(value_str, "motor2_value=\"%f\"", &value);
							value = MIN(MAX(value, -100), 100);
							value = (value / 100) * MOTOR_RANGE + MOTOR_CENTER;
							len = sprintf(cmd, "%d=%f\n", lg_motor2_id, value);
							write(fd, cmd, len);
						}

						value_str = strstr(xml, "motor3_value=");
						if (value_str) {
							char cmd[256];
							float value;
							int len;
							sscanf(value_str, "motor3_value=\"%f\"", &value);
							value = MIN(MAX(value, -100), 100);
							value = (value / 100) * MOTOR_RANGE + MOTOR_CENTER;
							len = sprintf(cmd, "%d=%f\n", lg_motor3_id, value);
							write(fd, cmd, len);
						}

						xmp = false;
						free(buff_xmp);
						buff_xmp = NULL;

						close(fd);
					}
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
}

int main(int argc, char *argv[]) {

	pthread_t transmit_thread;
	pthread_create(&transmit_thread, NULL, transmit_thread_func, (void*) NULL);

	pthread_t recieve_thread;
	pthread_create(&recieve_thread, NULL, recieve_thread_func, (void*) NULL);

	//wait exit command
	pthread_join(recieve_thread, NULL);

	return 0;
}
