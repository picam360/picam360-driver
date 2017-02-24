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

#include "picam360_driver.h"
#include "MotionSensor.h"

bool init() {
	ms_open();
	return true;
}

static float lg_quat[4];

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
	unsigned char *buff_trash = malloc(buff_size);
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
					int light_id = 34;
					int fd = open("/dev/pi-blaster", O_RDWR);
					if (fd > 0) {
						char *xml = buff_xmp + strlen(buff_xmp) + 1;

						char *value_str;
						value_str = strstr(xml, "light_value=");
						if (value_str) {
							char cmd[256];
							float value;
							sscanf(q_str, "light_value=\"%f\"", &value);
							int len = sprintf(cmd, "%d=%f\n", light_id, value);
							int len2 = write(fd, buf, len);
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
